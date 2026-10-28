#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grasp_once.py —— 单次抓取节点（替代厂商的 grab.py + put.py 那条链）

为什么要新写一个，而不是在 grab.py 上打补丁：
    见 `docs/抓取功能/01-抓取链路通读与断点清单.md` 的 G3/G4/G5/G7/G8/G17。
    简单说：grab.py 里"抓成功"是无条件置真的（G3）、按 car→clock→person 死序列触发（G5）、
    越界静默钳位（G8）、8 秒去抖会把重试吃掉（G17）。这些东西是嵌在它的结构里的，
    挤进去改比新写更容易出错。所以：**新文件并行存在，grab.py/put.py 一行不动**，
    需要退回老流程时随时能退。

和导航的关系（刻意保持完全兼容）：
    触发   ：ROS 参数 /grab 由 0/1 变化（nav.py 就是这么发的，一行不用改）
    完成   ：ROS 参数 /success = True（同样是 nav.py 等的那个）
    附加   ：/grasp_result 话题（std_msgs/String）逐阶段报进度，便于排查
             ~trigger 服务（std_srvs/Trigger）供手工单次触发，不必用导航

三种运行模式：
    1) 真抓（默认）：要 /dev/arm + 相机 + YOLO
    2) dry_run  ：不发串口、不动机械臂，只把"打算发什么"算出来打印
                  —— 上机第一件事就该是它，用来核对标定和逆解
    3) 离线自测：python3 grasp_math.py --selftest（不 import rospy，PC 上可跑）

用法：
    rosrun yolov8_ros grasp_once.py                       # 正常起（等 /grab）
    rosrun yolov8_ros grasp_once.py _dry_run:=true         # 空跑，只打印
    rosservice call /grasp_once/trigger                    # 手工抓一次
    rosparam set /grab 1                                   # 等价于让 nav.py 触发

安全（第一次上机务必先看）：
    - 本节点启动时会检查有没有 grab.py / put.py / arm_move.py / keyboard_grasp.py 还在跑
      —— 机械臂串口只能有一个属主，同时开会互相抢应答（G2）。
    - 像素越界、机械臂越限、逆解失败，一律【拒绝执行】并报清楚是哪一环，绝不硬凑。
"""

import json
import os
import sys
import threading
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

import grasp_math

try:
    from yolov8_ros.msg import Detection
except Exception as _e:      # 包里 msg 还没生成时给个明白的话
    rospy.logfatal("导入 yolov8_ros/Detection 失败（%s）：先把工作区编译一次（catkin_make）" % _e)
    raise


# ----------------------------------------------------------------------
# 假串口：dry_run 用。把所有指令记下来，方便和"真应该发什么"对照
# ----------------------------------------------------------------------
class FakeUart(object):
    def __init__(self):
        self.sent = []
        self.uart_get_ok = 0
        self.uart_receive_buf = ''

    def uart_send_str(self, s):
        self.sent.append(s)


# ----------------------------------------------------------------------
class GraspOnce(object):
    def __init__(self):
        # ---------------- 基本参数 ----------------
        self.dry_run = bool(rospy.get_param('~dry_run', False))
        self.baud = int(rospy.get_param('~baud', 115200))
        self.force = bool(rospy.get_param('~force', False))          # 忽略"别人占着串口"的检查

        # ---------------- 选目标 ----------------
        self.select_mode = rospy.get_param('~select', 'center')      # center | largest | arm
        self.target_class = rospy.get_param('~target_class', '')     # 空 = 任意类
        self.min_area = float(rospy.get_param('~min_area_px', 400))  # 太小（十几像素）的框不算
        self.img_w = int(rospy.get_param('~img_w', 640))
        self.img_h = int(rospy.get_param('~img_h', 480))
        self.det_max_age = float(rospy.get_param('~det_max_age', 0.8))   # 检测结果最多用 0.8 秒前的

        # ---------------- 抓取动作 ----------------
        self.z_grasp = float(rospy.get_param('~z_grasp', 10.0))
        self.z_lift = float(rospy.get_param('~z_lift', 150.0))
        self.offset_x = float(rospy.get_param('~offset_x', 0.0))
        self.offset_x_left = float(rospy.get_param('~offset_x_left', -6.0))  # X<=0 时厂商的那 -6mm
        self.offset_y = float(rospy.get_param('~offset_y', 20.0))            # 厂商 grab 的 +20
        self.prefer_alpha = rospy.get_param('~prefer_alpha', None)           # null = 厂商行为
        self.move_time = int(rospy.get_param('~move_time', 2000))
        self.lift_time = int(rospy.get_param('~lift_time', 2000))
        self.settle = float(rospy.get_param('~settle', 0.5))                 # 每个动作后额外等多久

        # ---- 2026-10-26：放置模式（不依赖视觉，放到固定的放物点）----
        self.put_x = float(rospy.get_param('~put_x', 0.0))
        self.put_y = float(rospy.get_param('~put_y', 200.0))
        self.put_z = float(rospy.get_param('~put_z', 25.0))

        # ---------------- 夹爪 ----------------
        self.grip_id = int(rospy.get_param('~gripper_id', 5))
        self.grip_open = int(rospy.get_param('~gripper_open_pwm', 1200))
        self.grip_close = int(rospy.get_param('~gripper_close_pwm', 1700))
        self.grip_time = int(rospy.get_param('~gripper_time', 1000))
        self.verify_gripper = bool(rospy.get_param('~verify_gripper', False))
        self.verify_delta = int(rospy.get_param('~verify_delta', 80))

        # ---------------- 运动学 / 标定 ----------------
        self.kin = grasp_math.Kinematics(
            float(rospy.get_param('~l0', 100)), float(rospy.get_param('~l1', 105)),
            float(rospy.get_param('~l2', 75)), float(rospy.get_param('~l3', 185)))
        self.limits = {
            "x_min": float(rospy.get_param('~limit_x_min', -140.0)),
            "x_max": float(rospy.get_param('~limit_x_max', 140.0)),
            "y_min": float(rospy.get_param('~limit_y_min', 80.0)),
            "y_max": float(rospy.get_param('~limit_y_max', 300.0)),
            "z_min": float(rospy.get_param('~limit_z_min', 0.0)),
            "z_max": float(rospy.get_param('~limit_z_max', 250.0)),
        }
        pkg = os.path.dirname(_HERE)
        self.calib_path = rospy.get_param('~calib_file',
                                          os.path.join(pkg, 'config', 'grasp_calib.json'))
        self.calib = grasp_math.load_calib(self.calib_path)
        if self.calib is None:
            self.calib = grasp_math.VendorInterp()
            rospy.logwarn("没有我们自己的标定文件（%s），回落到【厂商的 4 点插值】。\n"
                          "        这意味着相机一挪、机位一变就抓不准 —— 标定方法见 docs/抓取功能/。"
                          % self.calib_path)
        else:
            rospy.loginfo("标定：仿射 %d 点，rms=%.2f mm（%s）"
                          % (self.calib.n, self.calib.rms or -1, self.calib_path))

        # ---------------- 状态 ----------------
        self._det_lock = threading.Lock()
        self._detections = []          # [(t, dict), ...]
        self._last_det_wall = 0.0
        self._busy = threading.Event()
        self._last_grab_param = 0
        self._last_put_param = 0
        self._stop = False
        self._lock_file = None

        # ---------------- 串口 ----------------
        self.uart = self._open_uart()

        # ---------------- ROS 接口 ----------------
        self.pub = rospy.Publisher('/grasp_result', String, queue_size=10)
        self.sub = rospy.Subscriber('/yolov8_detections', Detection, self._on_detection, queue_size=30)
        self.srv = rospy.Service('~trigger', Trigger, self._on_trigger_srv)
        self.srv_put = rospy.Service('~put_trigger', Trigger, self._on_put_srv)
        self.timer = rospy.Timer(rospy.Duration(0.1), self._tick)
        rospy.on_shutdown(self._on_shutdown)

        rospy.set_param('/success', False)
        rospy.loginfo("grasp_once 就绪：%s，选目标=%s，类别=%s，标定=%s"
                      % ("dry_run（不发串口）" if self.dry_run else "真抓",
                         self.select_mode, self.target_class or "任意",
                         type(self.calib).__name__))
        rospy.loginfo("触发方式：rosparam set /grab 1  或  rosservice call %s/trigger"
                      % rospy.get_name())

    # ------------------------------------------------------------------
    # 串口
    # ------------------------------------------------------------------
    def _open_uart(self):
        if self.dry_run:
            rospy.logwarn("dry_run：用假串口，机械臂【不会动】。会把要发的指令打印出来。")
            return FakeUart()

        if not self.force and self._other_arm_users():
            raise RuntimeError(
                "机械臂串口可能被别的进程占着。同时开着会让两边抢应答（见断点 G2）。\n"
                "        请先 ./killall.sh，或确认后加 _force:=true 强开。")

        import WCXC_UartServer
        try:
            u = WCXC_UartServer.WCXC_UART(self.baud)
        except Exception as e:
            raise RuntimeError(
                "打不开机械臂串口（%s）。\n"
                "        ① ./7-grasp-check.sh 看 /dev/arm 在不在、属主对不对\n"
                "        ② 别名没建时不要改车内 udev，改用：export ABOT_ARM_PORT=/dev/ttyUSB2\n"
                % e)
        rospy.loginfo("机械臂串口已打开（%s @ %d）"
                      % (os.environ.get('ABOT_ARM_PORT', '/dev/arm'), self.baud))
        return u

    @staticmethod
    def _other_arm_users():
        """看看有没有厂商那套抓取节点还在跑。pgrep 不在就当作"没有"。"""
        import subprocess
        pat = 'grab.py|put.py|arm_move.py|keyboard_grasp'
        try:
            out = subprocess.check_output(['pgrep', '-af', pat],
                                          stderr=subprocess.DEVNULL).decode('utf-8', 'ignore')
        except Exception:
            return ''
        me = os.getpid()
        keep = []
        for line in out.strip().splitlines():
            parts = line.split(None, 1)
            if len(parts) == 2 and parts[0].isdigit() and int(parts[0]) != me:
                keep.append(line)
        return "\n".join(keep)

    def _send(self, text):
        rospy.loginfo("[ARM] -> %s" % text)
        self.uart.uart_send_str(text)

    def _read_servo(self, sid, timeout=1.0):
        """发 #NNNPRAD! 读回原始应答。返回 (raw, 最后一个整数 or None)。
        注意：舵机板不一定回包；回包格式手册没写死。所以这里只"尽力读"，
        读不到就返回 (''，None)，由调用方决定要不要当失败。"""
        if isinstance(self.uart, FakeUart):
            return '', None
        try:
            self.uart.uart_get_ok = 0
            self.uart.uart_receive_buf = ''
            self.uart.uart_send_str(grasp_math.fmt_read(sid))
            t0 = time.time()
            while time.time() - t0 < timeout:
                if getattr(self.uart, 'uart_get_ok', 0):
                    raw = getattr(self.uart, 'uart_receive_buf', '')
                    import re
                    nums = re.findall(r'-?\d+', raw)
                    return raw, (int(nums[-1]) if nums else None)
                time.sleep(0.02)
        except Exception as e:
            rospy.logwarn("读 %d 号舵机失败：%s" % (sid, e))
        return '', None

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _on_detection(self, msg):
        d = {"class_name": msg.class_name, "confidence": msg.confidence,
             "x1": msg.x1, "y1": msg.y1, "x2": msg.x2, "y2": msg.y2}
        now = time.time()
        with self._det_lock:
            self._detections.append((now, d))
            if len(self._detections) > 60:
                self._detections = self._detections[-60:]
            self._last_det_wall = now

    def _fresh_detections(self):
        now = time.time()
        with self._det_lock:
            return [d for (t, d) in self._detections if now - t <= self.det_max_age]

    def _tick(self, _evt):
        """10Hz 看 /grab 和 /put 的上升沿。用参数而不是话题，是为了和 nav.py 完全兼容。"""
        if self._busy.is_set():
            return
        for name, mode in (('/grab', 'grab'), ('/put', 'put')):
            v = rospy.get_param(name, 0)
            try:
                v = int(v)
            except Exception:
                v = 1 if v else 0
            last = getattr(self, '_last_' + mode + '_param', 0)
            setattr(self, '_last_' + mode + '_param', v)
            if v and not last:
                self._start(mode, name + ' 参数')

    def _on_trigger_srv(self, _req):
        if self._busy.is_set():
            return TriggerResponse(success=False, message="上一次抓取还没结束")
        self._start('grab', "service")
        return TriggerResponse(success=True, message="已开始，进度看 /grasp_result")

    def _on_put_srv(self, _req):
        if self._busy.is_set():
            return TriggerResponse(success=False, message="上一次动作还没结束")
        self._start('put', "service")
        return TriggerResponse(success=True, message="开始放置，进度看 /grasp_result")

    def _on_shutdown(self):
        self._stop = True
        try:
            if self._lock_file:
                self._lock_file.close()
        except Exception:
            pass

    def _start(self, mode, src):
        if self._busy.is_set():
            rospy.logwarn("上一次还没完，忽略这次触发（%s / %s）" % (mode, src))
            return
        self._busy.set()
        th = threading.Thread(target=self._run_guarded, args=(mode, src))
        th.daemon = True
        th.start()

    def _run_guarded(self, mode, src):
        t0 = time.time()
        try:
            res = self._run(mode, src)
        except Exception as e:
            import traceback
            traceback.print_exc()
            res = {"ok": False, "status": "exception", "reason": str(e)}
        finally:
            dp = self.dry_run
            rospy.set_param('/grab', 0)
            rospy.set_param('/put', 0)
            if res.get("ok") and not dp:
                rospy.set_param('/success', True)
            res["mode"] = mode
            res["elapsed_s"] = round(time.time() - t0, 2)
            res["dry_run"] = dp
            self.pub.publish(String(data=json.dumps(res, ensure_ascii=False)))
            rospy.loginfo("[RESULT] %s" % json.dumps(res, ensure_ascii=False))
            self._busy.clear()

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _run(self, mode, src):
        rospy.loginfo("========== 开始一次%s（来源：%s）%s =========="
                      % ("抓取" if mode == "grab" else "放置", src,
                         "  [dry_run]" if self.dry_run else ""))
        if mode == "put":
            return self._run_put()

        # ① 等一个"新鲜"的检测结果
        dmax = max(self.det_max_age, 1.0)
        t0 = time.time()
        dets = self._fresh_detections()
        while not dets and time.time() - t0 < dmax:
            if self._stop:
                return {"ok": False, "status": "aborted", "reason": "节点正在退出"}
            time.sleep(0.05)
            dets = self._fresh_detections()
        if not dets:
            return {"ok": False, "status": "no_detection",
                    "reason": "%.1f 秒内没收到 /yolov8_detections —— "
                              "先查相机话题对不对（断点 G1），再查有没有目标" % dmax}

        # ② 选目标
        sel, why = grasp_math.select_target(
            dets, mode=self.select_mode, img_w=self.img_w, img_h=self.img_h,
            target_class=self.target_class, min_area=self.min_area)
        if sel is None:
            return {"ok": False, "status": "no_target", "reason": why,
                    "n_det": len(dets),
                    "classes": sorted(set(d["class_name"] for d in dets))}
        det = sel["det"]
        rospy.loginfo("选中目标：%s conf=%.2f 中心=(%.0f,%.0f) 共 %d 个候选"
                      % (det["class_name"], det["confidence"], sel["u"], sel["v"],
                         sel["n_candidates"]))

        # ③ 规划（像素范围 → 换算 → 限幅 → 逆解，每步失败都能说清）
        params = {
            "z": self.z_grasp,
            "offset_x": self.offset_x,
            "offset_y": self.offset_y,
            "prefer_alpha": self.prefer_alpha,
            "limits": self.limits,
            "roi_margin": 0,
        }
        plan = grasp_math.plan_grasp(sel["u"], sel["v"], self.calib, self.kin, params)
        if not plan["ok"]:
            return {"ok": False, "status": "plan_failed", "stage": plan["stage"],
                    "reason": plan["reason"], "pixel": [sel["u"], sel["v"]],
                    "arm": [plan["x"], plan["y"]]}

        # 厂商在 X<=0 时多给 -6mm（腕部偏置，见 grab.py:170）。
        # 这里按同样条件叠加，并且【重走一遍限幅+逆解】—— 不能改个数就直接发。
        if plan["x"] <= 0 and self.offset_x_left:
            p2 = grasp_math.plan_grasp(
                sel["u"], sel["v"], self.calib, self.kin,
                dict(params, offset_x=self.offset_x + self.offset_x_left))
            if p2["ok"]:
                plan = p2
            else:
                rospy.logwarn("叠加 X 偏置后规划失败（%s），保持不加偏置的方案" % p2["reason"])

        rospy.loginfo("规划：像素(%.0f,%.0f) -> 机械臂(%.1f, %.1f, z=%.0f) alpha=%s pwm=%s"
                      % (sel["u"], sel["v"], plan["x"], plan["y"], plan["z"],
                         plan["alpha"], plan["pwm"]))

        if self.dry_run:
            return self._dry_run_report(det, sel, plan)

        # ④ 动机械臂：张爪 → 到位 → 合爪 → 抬起
        return self._execute(plan, det)

    def _wait(self, ms, why):
        t = ms / 1000.0 + self.settle
        rospy.loginfo("  等待 %.1f s（%s）" % (t, why))
        end = time.time() + t
        while time.time() < end:
            if self._stop:
                raise RuntimeError("节点正在退出，中止动作")
            time.sleep(0.05)

    def _run_put(self):
        """放置：走到固定放物点 -> 张爪 -> 抬起来。不看视觉，失败也能说清是哪一环。"""
        ok, why = grasp_math.check_arm(self.put_x, self.put_y, self.put_z, self.limits)
        if not ok:
            return {"ok": False, "status": "plan_failed", "stage": "arm_range", "reason": why}
        r = self.kin.plan(self.put_x, self.put_y, self.put_z,
                          prefer_alpha=self.prefer_alpha)
        if not r["ok"]:
            return {"ok": False, "status": "plan_failed", "stage": "ik", "reason": r["reason"]}
        p = r["pwm"]
        cmd = grasp_math.fmt_multi_map({0: p[0], 1: p[1], 2: p[2], 3: p[3], 4: p[0]},
                                       self.move_time)
        lift = self.kin.plan(self.put_x, self.put_y, self.z_lift,
                             prefer_alpha=self.prefer_alpha)
        lift_cmd = None
        if lift["ok"]:
            lp = lift["pwm"]
            lift_cmd = grasp_math.fmt_multi_map({0: lp[0], 1: lp[1], 2: lp[2], 3: lp[3],
                                                 4: lp[0]}, self.lift_time)

        if self.dry_run:
            rospy.loginfo("[dry_run] 放置会发：%s" % cmd)
            rospy.loginfo("[dry_run] 然后张爪 %s，再抬：%s"
                          % (grasp_math.fmt_one(self.grip_id, self.grip_open, self.grip_time),
                             lift_cmd))
            return {"ok": True, "status": "dry_run", "mode": "put",
                    "arm": [self.put_x, self.put_y, self.put_z], "alpha": r["alpha"],
                    "pwm": p,
                    "commands": [cmd,
                                 grasp_math.fmt_one(self.grip_id, self.grip_open,
                                                    self.grip_time),
                                 lift_cmd]}

        self._send(cmd)
        self._wait(self.move_time, "走到放物点")
        self._send(grasp_math.fmt_one(self.grip_id, self.grip_open, self.grip_time))
        self._wait(self.grip_time, "张爪放下")
        if lift_cmd:
            self._send(lift_cmd)
            self._wait(self.lift_time, "抬起")
        return {"ok": True, "status": "put_done",
                "arm": [self.put_x, self.put_y, self.put_z],
                "alpha": r["alpha"], "pwm": p, "lift_pwm": lift["pwm"] if lift["ok"] else None,
                "note": "Z=%s 是固定放物点，改 grasp_params.yaml 的 put_x/put_y/put_z"
                        % [self.put_x, self.put_y, self.put_z]}

    def _dry_run_report(self, det, sel, plan):
        cmds = []
        cmds.append(("① 张爪", grasp_math.fmt_one(self.grip_id, self.grip_open, self.grip_time)))
        cmds.append(("② 伸到目标", grasp_math.fmt_multi_map(
            {0: plan["pwm"][0], 1: plan["pwm"][1], 2: plan["pwm"][2],
             3: plan["pwm"][3], 4: plan["pwm"][0]}, self.move_time)))
        cmds.append(("③ 合爪", grasp_math.fmt_one(self.grip_id, self.grip_close, self.grip_time)))
        lift = self.kin.plan(plan["x"], plan["y"], self.z_lift,
                             prefer_alpha=self.prefer_alpha)
        if lift["ok"]:
            cmds.append(("④ 抬起", grasp_math.fmt_multi_map(
                {0: lift["pwm"][0], 1: lift["pwm"][1], 2: lift["pwm"][2],
                 3: lift["pwm"][3], 4: lift["pwm"][0]}, self.lift_time)))
        else:
            cmds.append(("④ 抬起", "不可达：%s" % lift["reason"]))
        rospy.loginfo("---- dry_run：会依次发这些（真跑时一模一样）----")
        for name, c in cmds:
            rospy.loginfo("  %-10s %s" % (name, c))
        rospy.loginfo("-----------------------------------------------------")
        return {"ok": True, "status": "dry_run",
                "target": det["class_name"],
                "pixel": [round(sel["u"], 1), round(sel["v"], 1)],
                "arm": [round(plan["x"], 1), round(plan["y"], 1), plan["z"]],
                "alpha": plan["alpha"], "pwm": plan["pwm"],
                "commands": [{"step": n, "cmd": c} for (n, c) in cmds],
                "note": "dry_run 不会置 /success，也不会动机械臂"}

    def _execute(self, plan, det):
        steps = []

        def wait(ms, extra=0.0, why=""):
            t = ms / 1000.0 + self.settle + extra
            rospy.loginfo("  等待 %.1f s（%s）" % (t, why))
            end = time.time() + t
            while time.time() < end:
                if self._stop:
                    raise RuntimeError("节点正在退出，中止动作")
                time.sleep(0.05)

        # ① 张爪
        self._send(grasp_math.fmt_one(self.grip_id, self.grip_open, self.grip_time))
        wait(self.grip_time, why="张爪")
        steps.append("open")

        # ② 伸到目标（0~3 号按 IK，4 号腕部跟 0 号，与厂商一致）
        cmd = grasp_math.fmt_multi_map(
            {0: plan["pwm"][0], 1: plan["pwm"][1], 2: plan["pwm"][2],
             3: plan["pwm"][3], 4: plan["pwm"][0]}, self.move_time)
        self._send(cmd)
        wait(self.move_time, why="伸到目标")
        steps.append("reach")

        # ③ 合爪（可选回读，做"夹到了没有"的粗判）
        self._send(grasp_math.fmt_one(self.grip_id, self.grip_close, self.grip_time))
        wait(self.grip_time, why="合爪")
        steps.append("close")
        g_raw, g_ang = self._read_servo(self.grip_id, timeout=0.8)
        if g_raw:
            rospy.loginfo("  夹爪回读：%r -> %s" % (g_raw, g_ang))
        else:
            rospy.logwarn("  夹爪回读无回应（不一定是故障：舵机板可能不回 RAD 查询）")

        # ④ 抬起
        lift = self.kin.plan(plan["x"], plan["y"], self.z_lift, prefer_alpha=self.prefer_alpha)
        if not lift["ok"]:
            # 抬不起来就用原姿态原地保持，不要瞎发
            return {"ok": False, "status": "lift_unreachable", "reason": lift["reason"],
                    "pixel": None, "arm": [plan["x"], plan["y"], self.z_lift],
                    "steps": steps}
        self._send(grasp_math.fmt_multi_map(
            {0: lift["pwm"][0], 1: lift["pwm"][1], 2: lift["pwm"][2],
             3: lift["pwm"][3], 4: lift["pwm"][0]}, self.lift_time))
        wait(self.lift_time, why="抬起")
        steps.append("lift")

        status = "grasped"
        note = "动作已全部下发。注意：这仍然只是'指令走完了'，不是'抓到了'——"
        if self.verify_gripper and g_ang is not None:
            note += "夹爪回读 %d。" % g_ang
        else:
            note += "verify_gripper 未开启（见断点 G3）。"

        return {"ok": True, "status": status, "target": det["class_name"],
                "arm": [round(plan["x"], 1), round(plan["y"], 1), plan["z"]],
                "pwm": plan["pwm"], "lift_pwm": lift["pwm"],
                "gripper_readback": g_ang, "steps": steps, "note": note}


# ----------------------------------------------------------------------
def main():
    # ★ 不 anonymous：服务名要稳定（/grasp_once/trigger），
    #   而且机械臂串口天生只能有一个属主，本来就不允许开第二份。
    rospy.init_node('grasp_once', anonymous=False)
    try:
        node = GraspOnce()
    except RuntimeError as e:
        rospy.logfatal("%s" % e)
        sys.exit(1)
    rospy.loginfo("等待触发中……（Ctrl-C 退出）")
    rospy.spin()


if __name__ == '__main__':
    main()
