#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""template_orb.py —— 纯 Python 的 ORB 模板匹配，顶替 C++ 的 find_object_2d

为什么要有它（2026-10-28 晚结论）：
    本机 Jetson 上同时装了两套 OpenCV：ROS 的 cv_bridge 用 4.2，
    而 libfind_object_2d.so 用 4.5 编译，两者同名符号冲突：
        SIFT（只有 4.5 有）调 cv::resize 却绑到 4.2  -> SIGSEGV
        换 ORB（两边都有）绑到 4.2，对象交回 4.5 代码  -> FATAL detector_ != 0
    Python 的 cv2 是独立绑定，和 cv_bridge 不共享任何 C++ 对象（只传 bytes），
    所以天然绕开这个 ABI 问题。详见 交接日志-2026-10-28.md 第 8 节。

输出契约（必须与 find_object_2d 一致，ai_locate.py 才吃得下）：
    话题 /objects，类型 std_msgs/Float32MultiArray，每个目标 12 个 float：
        [id, 模板宽W, 模板高H, m11 m12 m13 m21 m22 m23 m31 m32 m33]
    m 是「模板坐标 -> 图像坐标」的单应矩阵，Qt QTransform 顺序：
        X = (m11*x + m21*y + m31) / (m13*x + m23*y + m33)
        Y = (m12*x + m22*y + m32) / (m13*x + m23*y + m33)
    ai_locate.py 拿模板四角 (0,0)(W,0)(0,H)(W,H) 去投影出 bbox，
    所以方向必须是「模板->图像」，反了会得到离谱的大框。

用法：
    rosrun yolov8_ros template_orb.py
    rosrun yolov8_ros template_orb.py _min_inliers:=15 _n_features:=800
    ./ai.sh v0 dry orb          # 走整条链（ai_grasp_orb.launch）

离线自检（不需要 ROS、不碰相机，车上/PC 都能跑）：
    python3 template_orb.py --selftest
    拿 9 张模板各自当输入图跑一遍，打印每条命中的 inliers，
    并给出「哪几张会串号」的矩阵 —— 苹果(4) / 红枣(9) 是重点怀疑对象。
"""

import os
import sys
import time

import numpy as np
import cv2

FLOATS_PER_OBJECT = 12


# ----------------------------------------------------------------------
# 模板装载
# ----------------------------------------------------------------------
def list_templates(objects_path):
    """只认「纯数字文件名」。find_object_2d 的规矩：文件名数字 = id，且 0 表示自动分配。"""
    items = []
    if not os.path.isdir(objects_path):
        return items
    for name in sorted(os.listdir(objects_path)):
        stem, ext = os.path.splitext(name)
        if ext.lower() not in ('.png', '.jpg', '.jpeg', '.bmp'):
            continue
        if not stem.isdigit():
            continue
        oid = int(stem)
        if oid <= 0:
            print('[!] 跳过 %s：id 必须 >0（0 会被上游当成"自动分配"，串号）' % name)
            continue
        items.append((oid, os.path.join(objects_path, name)))
    return items


def default_objects_path():
    """src/yolov8_ros/script/ 往上两级 -> src/abot_find/object_task"""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, '..', '..', 'abot_find', 'object_task'))


# ----------------------------------------------------------------------
# 匹配核心（不依赖 ROS，方便离线自检）
# ----------------------------------------------------------------------
class TemplateMatcher(object):
    def __init__(self, objects_path, n_features=600, nndr=0.75, min_inliers=12):
        self.nndr = float(nndr)
        self.min_inliers = int(min_inliers)
        self.orb = cv2.ORB_create(nfeatures=int(n_features))
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.templates = []
        self._load(objects_path)

    def _load(self, objects_path):
        items = list_templates(objects_path)
        if not items:
            raise RuntimeError('模板目录里没有任何「数字.png」：%s' % objects_path)

        for oid, path in items:
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print('[!!] 读不出来（路径或编码问题）：%s' % path)
                continue
            kp, des = self.orb.detectAndCompute(img, None)
            if des is None or len(kp) < 8:
                print('[!!] 特征太少（%d 个），这张没进库：%s'
                      % (0 if kp is None else len(kp), os.path.basename(path)))
                continue
            h, w = img.shape[:2]
            self.templates.append({
                'id': oid,
                'path': path,
                'w': float(w),
                'h': float(h),
                'pts': np.float32([k.pt for k in kp]),
                'des': des,
            })
            print('[ok] 模板 %d  %dx%d  %d 个特征  %s'
                  % (oid, w, h, len(kp), os.path.basename(path)))

        if not self.templates:
            raise RuntimeError('一张模板都没加载成功，检查目录：%s' % objects_path)

    def detect(self, gray, max_objects=3, min_area_px=400.0, img_size=None):
        """返回 [(id, w, h, H, inliers, good)]，按 inliers 从多到少。"""
        kp, des = self.orb.detectAndCompute(gray, None)
        if des is None or len(kp) < 8:
            return []
        scene_pts = np.float32([k.pt for k in kp])

        found = []
        for t in self.templates:
            try:
                pairs = self.bf.knnMatch(t['des'], des, k=2)
            except cv2.error:
                continue

            good = []
            for pair in pairs:
                if len(pair) < 2:
                    continue
                best, second = pair
                # Lowe 比值检验：最佳比次佳小很多，才算可信匹配
                if best.distance < self.nndr * second.distance:
                    good.append(best)

            if len(good) < max(4, self.min_inliers):
                continue

            src = np.float32([t['pts'][m.queryIdx] for m in good])
            dst = np.float32([scene_pts[m.trainIdx] for m in good])

            try:
                H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
            except cv2.error:
                continue
            if H is None:
                continue
            inliers = int(mask.sum()) if mask is not None else 0
            if inliers < self.min_inliers:
                continue

            # 面积/合理性过滤：把模板四角投到图像上，框太小或离谱的丢掉
            corners = cv2.perspectiveTransform(
                np.float32([[[0, 0]], [[t['w'], 0]], [[0, t['h']]], [[t['w'], t['h']]]]), H)
            xs = corners[:, 0, 0]
            ys = corners[:, 0, 1]
            area = float((xs.max() - xs.min()) * (ys.max() - ys.min()))
            if area < min_area_px:
                continue
            if img_size is not None:
                iw, ih = img_size
                if xs.max() < 0 or ys.max() < 0 or xs.min() > iw or ys.min() > ih:
                    continue

            found.append((t['id'], t['w'], t['h'], H, inliers, len(good)))

        found.sort(key=lambda x: -x[4])
        return found[:max_objects]


def to_float_array(found):
    """把 detect() 的结果摊成 /objects 的 12-float 格式。"""
    data = []
    for oid, w, h, H, _inliers, _good in found:
        data.append(float(oid))
        data.append(float(w))
        data.append(float(h))
        # H.T 摊平 = Qt QTransform 的 m11 m12 m13 m21 m22 m23 m31 m32 m33
        data.extend([float(H[0, 0]), float(H[1, 0]), float(H[2, 0]),
                     float(H[0, 1]), float(H[1, 1]), float(H[2, 1]),
                     float(H[0, 2]), float(H[1, 2]), float(H[2, 2])])
    return data


def draw_debug(gray_bgr, found):
    for oid, w, h, H, inliers, _good in found:
        corners = cv2.perspectiveTransform(
            np.float32([[[0, 0]], [[w, 0]], [[w, h]], [[0, h]]]), H).astype(np.int32)
        cv2.polylines(gray_bgr, [corners], True, (0, 255, 0), 2)
        x, y = corners[0][0]
        cv2.putText(gray_bgr, 'id%d(%d)' % (oid, inliers), (int(x), int(y) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return gray_bgr


# ----------------------------------------------------------------------
# 离线自检：不需要 ROS
# ----------------------------------------------------------------------
def selftest(objects_path, n_features, nndr, min_inliers):
    print('=== 装载模板（%s）===' % objects_path)
    matcher = TemplateMatcher(objects_path, n_features, nndr, min_inliers)
    print('\n共装载 %d 张\n' % len(matcher.templates))

    print('=== 自匹配：每张模板当输入图，理应以高分认回自己 ===')
    print('    （若某张认不回自己，说明 ORB 在这张图上特征不够，得换图）')
    ok = 0
    for t in matcher.templates:
        img = cv2.imread(t['path'], cv2.IMREAD_GRAYSCALE)
        if img is None:
            print('  [!!] id=%d 读不出来' % t['id'])
            continue
        h, w = img.shape[:2]
        found = matcher.detect(img, max_objects=5, min_area_px=0.0, img_size=(w, h))
        got = [(f[0], f[4]) for f in found]
        self_hit = [g for g in got if g[0] == t['id']]
        if self_hit:
            ok += 1
            mark = 'OK '
        else:
            mark = '!! '
        others = [(i, n) for (i, n) in got if i != t['id']]
        print('  %sid=%-2d 命中=%s%s' % (mark, t['id'], self_hit or '无',
                                        ('   串到了 %s' % others) if others else ''))
    print('\n自匹配成功 %d/%d\n' % (ok, len(matcher.templates)))

    print('=== 串号对照（重点看 4 苹果 / 9 红枣）===')
    print('    做法：拿 id=A 的图当输入，看它会不会同时认成 B')
    for a in matcher.templates:
        img = cv2.imread(a['path'], cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        h, w = img.shape[:2]
        found = matcher.detect(img, max_objects=5, min_area_px=0.0, img_size=(w, h))
        ids = [f[0] for f in found]
        if len(ids) > 1:
            print('  id=%-2d 同时命中 %s   <- 这两张模板太像' % (a['id'], ids))
    print('\n=== 完了。上面没有 "!!" 和串号行，就可以上机试 ===')


# ----------------------------------------------------------------------
# ROS 节点
# ----------------------------------------------------------------------
def run_node():
    import rospy
    from std_msgs.msg import Float32MultiArray
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rospy.init_node('template_orb', anonymous=False)

    objects_path = rospy.get_param('~objects_path', default_objects_path())
    n_features = rospy.get_param('~n_features', 600)
    nndr = rospy.get_param('~nndr', 0.75)
    min_inliers = rospy.get_param('~min_inliers', 12)
    max_objects = rospy.get_param('~max_objects', 3)
    min_area = rospy.get_param('~min_area_px', 400.0)
    image_topic = rospy.get_param('~image_topic', '/usb_cam2/image_raw')
    objects_topic = rospy.get_param('~objects_topic', '/objects')
    debug_topic = rospy.get_param('~debug_image_topic', '')   # 空 = 不发调试图
    log_period = float(rospy.get_param('~log_period', 1.0))

    matcher = TemplateMatcher(objects_path, n_features, nndr, min_inliers)

    pub = rospy.Publisher(objects_topic, Float32MultiArray, queue_size=10)
    pub_dbg = rospy.Publisher(debug_topic, Image, queue_size=1) if debug_topic else None
    bridge = CvBridge()

    state = {'last_log': 0.0, 'frames': 0, 'fps_t0': time.time()}

    def callback(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:      # noqa: BLE001 - cv_bridge 抛的异常类型很杂
            rospy.logerr_throttle(5.0, 'cv_bridge 转换失败：%s' % e)
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        found = matcher.detect(gray, max_objects=max_objects,
                               min_area_px=min_area, img_size=(w, h))

        out = Float32MultiArray()
        out.data = to_float_array(found)
        pub.publish(out)

        state['frames'] += 1
        now = time.time()
        if now - state['last_log'] >= log_period:
            state['last_log'] = now
            fps = state['frames'] / max(1e-6, now - state['fps_t0'])
            state['frames'] = 0
            state['fps_t0'] = now
            if found:
                rospy.loginfo('[orb] %s  (%.1f Hz)'
                              % (', '.join('id%d/%d' % (f[0], f[4]) for f in found), fps))
            else:
                rospy.loginfo('[orb] 无目标  (%.1f Hz)' % fps)

        if pub_dbg is not None:
            try:
                pub_dbg.publish(bridge.cv2_to_imgmsg(draw_debug(frame, found), 'bgr8'))
            except Exception:       # noqa: BLE001
                pass

    rospy.Subscriber(image_topic, Image, callback, queue_size=1, buff_size=2 ** 24)
    rospy.loginfo('template_orb 就绪：%d 张模板  %s -> %s'
                  % (len(matcher.templates), image_topic, objects_topic))
    rospy.loginfo('参数：n_features=%s nndr=%s min_inliers=%s max_objects=%s'
                  % (n_features, nndr, min_inliers, max_objects))
    rospy.spin()


def main():
    if '--selftest' in sys.argv:
        args = [a for a in sys.argv[1:] if not a.startswith('--')]
        path = args[0] if args else default_objects_path()
        selftest(path, n_features=600, nndr=0.75, min_inliers=12)
        return
    run_node()


if __name__ == '__main__':
    main()
