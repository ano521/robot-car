#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grasp_calib.py —— 现场标定工具：把「画面上的像素点」和「机械臂能抓到的位置」对上

为什么必须自己标：
    grab.py 里那 4 个点（像素 187,362 ↔ -88,170 等）是**厂商在厂商机位**量的。
    相机挪过、俯仰变过、换了桌子，它们就全错。而且它只是两点一维插值，
    连旋转都矫正不了（离线复算显示：同一像素点，它自己算出的两套坐标差 3~6 cm，
    见 `docs/抓取功能/02-离线逆运动学复算.md`）。

怎么标（推荐顺序）：
    0) 先把相机固定死、车停稳、桌面清空；用 `keyboard_grasp.launch` 能手动让臂走到任意 (x,y)
    1) 在桌面上贴 3×3 共 9 个标记点（间距 8~10 cm），落在画面中间区域
    2) 每个点做两件事：
         a. 在画面上点它 —— 拿到像素 (u,v)
            `rosrun yolov8_ros grasp_calib.py --click`
         b. 用手动方式把爪子中心移到这个点上 —— 记下当时的机械臂坐标 (x, y)
            （keyboard_grasp 的 rqt_reconfigure 里那组 kms_x/kms_y 就是）
       然后记一条：`--add u v x y`
    3) 全部记完：`--fit` —— 拟合仿射变换、打印残差、写到 grasp_calib.json
    4) 抓取节点（grasp_once.py）下次启动就会用它

    ★ 残差 RMS 超过 10 mm 不要接受 —— 要么点没对准，要么相机被碰了。

命令：
    --list                          看已经记了几条、每条残差多少
    --add U V X Y [--note ...]      记一条（U,V 像素；X,Y 机械臂 mm）
    --click                         开窗口，鼠标点一下打印 (u,v)
    --fit [--save] [--min-rms 10]   拟合；默认残差合格才写文件
    --show                          显示当前 grasp_calib.json 的内容
    --del N                         删掉第 N 条（从 1 开始）
"""

import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import grasp_math

PKG = os.path.dirname(_HERE)
POINTS = os.path.join(PKG, 'config', 'grasp_calib_points.json')
OUT = os.path.join(PKG, 'config', 'grasp_calib.json')


def _load_points():
    if not os.path.exists(POINTS):
        return []
    try:
        with open(POINTS, encoding='utf-8') as f:
            d = json.load(f)
        return [tuple(p) for p in d.get('points', [])]
    except Exception as e:
        print("[!!] 读 %s 失败：%s" % (POINTS, e))
        return []


def _save_points(pts, notes=None):
    d = {"points": [list(p[:4]) for p in pts]}
    if notes:
        d["notes"] = notes
    os.makedirs(os.path.dirname(POINTS), exist_ok=True)
    with open(POINTS, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=2)


def cmd_list():
    pts = _load_points()
    print("标定点文件：%s" % POINTS)
    if not pts:
        print("  （还没有任何标定点）")
        print("  先 --click 拿像素，再 --add u v x y")
        return 0
    print("  共 %d 条：" % len(pts))
    print("  %-3s %8s %8s   %9s %9s" % ("#", "u", "v", "x(mm)", "y(mm)"))
    for i, p in enumerate(pts, 1):
        print("  %-3d %8.1f %8.1f   %9.1f %9.1f" % (i, p[0], p[1], p[2], p[3]))
    if len(pts) >= 3:
        try:
            cal, rep = grasp_math.AffineCalib.fit(pts)
            print()
            print("  当前这批点若不改直接拟合：RMS=%.2f mm  最大=%.2f mm  系数=%s"
                  % (rep["rms_mm"], rep["max_mm"], rep["coeffs"]))
        except ValueError as e:
            print("  [!!] 拟合不出来：%s" % e)
    if len(pts) < 6:
        print("  [~~] 建议凑到 ≥6 条（3×3 网格用 6 条拟合靠得住，9 条最好）")
    return 0


def cmd_add(argv):
    if len(argv) < 4:
        print("用法：--add U V X Y [--note 说明]")
        return 2
    try:
        u, v, x, y = [float(t) for t in argv[:4]]
    except ValueError:
        print("[!!] 四个数必须是数字：u v x y")
        return 2
    note = ''
    if '--note' in argv:
        note = ' '.join(argv[argv.index('--note') + 1:])
    pts = _load_points()
    pts.append((u, v, x, y))
    _save_points(pts)
    print("[OK] 已记第 %d 条：(%.1f,%.1f) -> (%.1f, %.1f) %s"
          % (len(pts), u, v, x, y, note))
    if len(pts) >= 3:
        try:
            _, rep = grasp_math.AffineCalib.fit(pts)
            print("     现在整体 RMS=%.2f mm（点越多越可信）" % rep["rms_mm"])
        except ValueError as e:
            print("     [~~] 还拟合不出来：%s" % e)
    return 0


def cmd_del(argv):
    if not argv:
        print("用法：--del N")
        return 2
    pts = _load_points()
    n = int(argv[0])
    if not (1 <= n <= len(pts)):
        print("[!!] 没有第 %d 条（共 %d 条）" % (n, len(pts)))
        return 2
    gone = pts.pop(n - 1)
    _save_points(pts)
    print("[OK] 删掉了 (%.1f,%.1f)->(%.1f,%.1f)，剩 %d 条" % (gone + (len(pts),)))
    return 0


def cmd_fit(argv):
    pts = _load_points()
    if len(pts) < 3:
        print("[!!] 至少要 3 条才能拟合，现在 %d 条" % len(pts))
        return 1
    try:
        cal, rep = grasp_math.AffineCalib.fit(pts)
    except ValueError as e:
        print("[!!] 拟合失败：%s" % e)
        return 1

    print("拟合结果（仿射：x=a*u+b*v+c, y=d*u+e*v+f）")
    print("  用了 %d 个点" % rep["n"])
    print("  RMS  = %.2f mm" % rep["rms_mm"])
    print("  MAX  = %.2f mm" % rep["max_mm"])
    print("  每点残差 = %s" % rep["per_point_mm"])
    print("  系数 = %s" % rep["coeffs"])

    min_rms = 10.0
    if '--min-rms' in argv:
        min_rms = float(argv[argv.index('--min-rms') + 1])
    force = '--force' in argv
    save = '--save' in argv or force

    if rep["rms_mm"] > min_rms:
        print()
        print("  [!!] 残差 %.2f mm 超过阈值 %.1f mm —— 先别用。" % (rep["rms_mm"], min_rms))
        print("       常见原因：标记点没对准 / 车或相机动过 / 臂长参数 L0..L3 不对。")
        print("       确实要用就加 --force。")
        if not force:
            return 1

    if save:
        cal.source = "grasp_calib.py --fit"
        d = grasp_math.save_calib(OUT, cal)
        print()
        print("[OK] 已写 %s" % OUT)
        print("     %s" % json.dumps(d, ensure_ascii=False))
        print("     下次启动 grasp_once.py 会自动用它（日志里会打 '仿射 N 点 rms=...'）")
    else:
        print()
        print("（没有 --save，所以没写文件。要写入就加 --save）")
    return 0


def cmd_show():
    print("标定文件：%s" % OUT)
    if not os.path.exists(OUT):
        print("  （不存在 —— 抓取节点会回落到厂商的 4 点插值）")
        return 0
    with open(OUT, encoding='utf-8') as f:
        print(f.read())
    return 0


def cmd_click():
    """开窗口点像素。只读画面，不动机械臂。"""
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    state = {"img": None}
    bridge = CvBridge()

    def cb(msg):
        try:
            state["img"] = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("[!!] 图像转换失败：%s" % e)

    rospy.init_node('grasp_calib_click', anonymous=True)
    topic = rospy.get_param('~image_topic', '/usb_cam2/image_raw')
    rospy.Subscriber(topic, Image, cb, queue_size=1)
    print("订阅 %s，等第一帧……（Ctrl-C 退出）" % topic)

    t0 = rospy.get_time()
    while state["img"] is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
        if rospy.get_time() - t0 > 10:
            print("[!!] 10 秒没等到图像。先确认相机在发 %s（./7-grasp-check.sh 第 4 节）" % topic)
            return 1
    if state["img"] is None:
        return 1

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("  像素 (u,v) = (%d, %d)     ← 拿这个去 --add %d %d X Y" % (x, y, x, y))

    win = 'grasp_calib  (click a marker; ESC to quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    print("窗口里点左键 = 打印该像素；按 ESC 或 q 退出")
    while not rospy.is_shutdown():
        img = state["img"]
        if img is None:
            rospy.sleep(0.05)
            continue
        cv2.imshow(win, img)
        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord('q')):
            break
        rospy.sleep(0.01)
    cv2.destroyAllWindows()
    return 0


def main(argv):
    if not argv:
        print(__doc__)
        return 0
    a = argv[0]
    if a == '--list':
        return cmd_list()
    if a == '--add':
        return cmd_add(argv[1:])
    if a == '--del':
        return cmd_del(argv[1:])
    if a == '--fit':
        return cmd_fit(argv[1:])
    if a == '--show':
        return cmd_show()
    if a == '--click':
        return cmd_click()
    if a in ('-h', '--help'):
        print(__doc__)
        return 0
    print("未知参数：%s" % a)
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
