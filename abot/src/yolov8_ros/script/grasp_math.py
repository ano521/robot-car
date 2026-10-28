#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取用的纯数学模块 —— 逆运动学 + 像素↔机械臂 标定。

★ 设计约束（很重要，别破坏）：
   1. 本文件【不 import rospy、不 import serial、不 import cv2、不 import numpy】。
      原因：它要能在 PC 上（Windows / 没有 ROS 的机器）跑 `--selftest` 做回归测试。
      抓取逻辑错了代价是撞机械臂，所以核心算术必须有"不接硬件也能验"的通道。
   2. 逆运动学部分是从 src/yolov8_ros/script/z_kinematics.py **逐行复刻**的，
      连 7 个错误码的含义都不改。理由：厂商的臂是实机调过的，
      我这里只做"多一个选择"，不做"重新发明"。
   3. 所有"越界"一律【拒绝执行】，绝不静默钳位。
      （厂商 grab.py 用 np.interp，越界会静默取端点 —— 那就是"臂奔向极限位置还报正常"的来源。）

用法（PC 上可跑）：
    python3 grasp_math.py --selftest        # 回归自测，全 PASS 才算没改坏
    python3 grasp_math.py --demo            # 打印当前标定文件 + 几个点的换算结果
"""

import json
import math
import os

PI = 3.14159265358979

# ----------------------------------------------------------------------
# 一、逆运动学（复刻 z_kinematics.py）
# ----------------------------------------------------------------------

# 七个错误码，含义沿用原码（原码没写注释，这里是按判断顺序反推的）
ERR_TEXT = {
    0: "OK",
    1: "目标低于底座平面（z < -l0）",
    2: "超出 L1+L2 的臂展（够不着）",
    3: "关节角反解 acos 越界（几何不成立）",
    4: "theta5 超出 [0,180]（1 号舵机转不到位）",
    5: "theta4 的 acos 越界",
    6: "theta4 超出 ±135（2 号舵机行程不够）",
    7: "theta3 超出 ±90（3 号舵机行程不够）",
}


class Kinematics(object):
    """ABOT 五轴臂（实际用到 0~3 号舵机 + 5 号夹爪）。长度单位 mm。"""

    def __init__(self, l0=100, l1=105, l2=75, l3=185):
        self.l0 = l0 * 10.0
        self.l1 = l1 * 10.0
        self.l2 = l2 * 10.0
        self.l3 = l3 * 10.0

    # -- 单次反解 ------------------------------------------------------
    def solve(self, x, y, z, alpha):
        """返回 (err, pwm)。err==0 时 pwm=[s0,s1,s2,s3]，否则 pwm=None。"""
        l0, l1, l2, l3 = self.l0, self.l1, self.l2, self.l3
        x, y, z = x * 10.0, y * 10.0, z * 10.0

        if x == 0 and y != 0:
            theta6 = 0.0
        elif x > 0 and y == 0:
            theta6 = 90.0
        elif x < 0 and y == 0:
            theta6 = -90.0
        else:
            if y == 0:
                return 3, None          # x==0 且 y==0：原码会 ZeroDivisionError，这里明确拒绝
            theta6 = math.degrees(math.atan(x / y))

        y = math.sqrt(x * x + y * y)
        y = y - l3 * math.cos(alpha * PI / 180.0)
        z = z - l0 - l3 * math.sin(alpha * PI / 180.0)

        if z < -l0:
            return 1, None
        hyp = math.sqrt(y * y + z * z)
        if hyp == 0:
            return 3, None
        if hyp > (l1 + l2):
            return 2, None

        ccc = math.acos(max(-1.0, min(1.0, y / hyp)))
        bbb = (y * y + z * z + l1 * l1 - l2 * l2) / (2 * l1 * hyp)
        if bbb > 1 or bbb < -1:
            return 3, None

        zf = -1 if z < 0 else 1
        theta5 = math.degrees(ccc * zf + math.acos(bbb))
        if theta5 > 180.0 or theta5 < 0.0:
            return 4, None

        aaa = -(y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2)
        if aaa > 1 or aaa < -1:
            return 5, None
        theta4 = 180.0 - math.degrees(math.acos(aaa))
        if theta4 > 135.0 or theta4 < -135.0:
            return 6, None

        theta3 = alpha - theta5 + theta4
        if theta3 > 90.0 or theta3 < -90.0:
            return 7, None

        ang = [theta6, theta5 - 90.0, theta4, theta3]
        pwm = [int(1500 - 2000.0 * ang[0] / 270.0),
               int(1500 + 2000.0 * ang[1] / 270.0),
               int(1500 + 2000.0 * ang[2] / 270.0),
               int(1500 + 2000.0 * ang[3] / 270.0)]
        return 0, pwm

    # -- 选角 + 全量校验 ------------------------------------------------
    def plan(self, x, y, z, prefer_alpha=None, pwm_min=500, pwm_max=2500):
        """
        在 alpha ∈ [-135, -1] 里找可行解。

        prefer_alpha=None  → 取【最负】的可行值（＝厂商 grab.py 的行为，默认保持兼容）
        prefer_alpha=-45   → 取最接近它的可行值（＝ z_kinematics.py 注释里说的"比较好"区间）

        返回 dict:
            ok       : True/False
            alpha    : 选中的角（失败为 None）
            pwm      : [s0,s1,s2,s3]（失败为 None）
            err      : 失败原因码（成功为 0）
            reason   : 可读的失败原因
            best_err : 所有尝试里最"不致命"的错误码（用来解释为什么不行）
            feasible : 可行 alpha 的个数
        """
        feasible = []
        for a in range(-135, 0):
            err, pwm = self.solve(x, y, z, a)
            if err == 0:
                feasible.append((a, pwm))

        if not feasible:
            # 解释失败：取范围 [3,7] 里最先出现的错误码（1/2 是"绝对够不着"，
            # 3~7 是"姿态不对"，多数时候是姿态问题，所以优先报 3 之后那类）
            best = None
            for a in (-90, -60, -45, -30, -75):
                err, _ = self.solve(x, y, z, a)
                if err in (3, 4, 5, 6, 7):
                    best = err
                    break
            if best is None:
                err, _ = self.solve(x, y, z, -45)
                best = err
            return {"ok": False, "alpha": None, "pwm": None, "err": best,
                    "reason": ERR_TEXT.get(best, "未知"), "best_err": best,
                    "feasible": 0}

        if prefer_alpha is None:
            alpha, pwm = min(feasible, key=lambda t: t[0])       # 最负（厂商行为）
        else:
            alpha, pwm = min(feasible, key=lambda t: abs(t[0] - prefer_alpha))

        bad = [i for i, v in enumerate(pwm) if not (pwm_min <= v <= pwm_max)]
        if bad:
            return {"ok": False, "alpha": alpha, "pwm": pwm, "err": 8,
                    "reason": "PWM 越界：舵机 %s 不在 [%d,%d]" % (bad, pwm_min, pwm_max),
                    "best_err": 8, "feasible": len(feasible)}

        return {"ok": True, "alpha": alpha, "pwm": pwm, "err": 0,
                "reason": "OK", "best_err": 0, "feasible": len(feasible)}

    def reachable(self, x, y, z, prefer_alpha=None):
        return self.plan(x, y, z, prefer_alpha)["ok"]


# ----------------------------------------------------------------------
# 二、像素 → 机械臂 标定
# ----------------------------------------------------------------------

class VendorInterp(object):
    """
    厂商 grab.py 里那套「两点线性插值」，原样保留。
    作用：① 没有我们自己的标定文件时的兜底；② 和我们的新标定做对照。
    注意 np.interp 的钳位语义我**刻意复刻**了（越界取端点），
    但 caller 必须先自己判范围 —— check_pixel() 就是干这个的。
    """

    def __init__(self, c1_im=(187, 362), c1_mc=(-88, 170),
                 c2_im=(492, 119), c2_mc=(110, 260)):
        self.x_im = [c1_im[0], c2_im[0]]
        self.x_mc = [c1_mc[0], c2_mc[0]]
        self.y_im = [c2_im[1], c1_im[1]]
        self.y_mc = [c2_mc[1], c1_mc[1]]

    @staticmethod
    def _interp(v, xs, ys):
        if v <= xs[0]:
            return ys[0]
        if v >= xs[-1]:
            return ys[-1]
        t = (v - xs[0]) / (xs[-1] - xs[0])
        return ys[0] + t * (ys[-1] - ys[0])

    def map(self, u, v):
        return (self._interp(u, self.x_im, self.x_mc),
                self._interp(v, self.y_im, self.y_mc))

    def roi(self):
        return (min(self.x_im), min(self.y_im), max(self.x_im), max(self.y_im))


class AffineCalib(object):
    """
    我们自己的标定：仿射映射 (u,v) -> (x,y)，最小二乘拟合。
        x = a*u + b*v + c
        y = d*u + e*v + f
    比厂商的两点插值多能矫正：旋转、缩放不一致、相机俯仰造成的剪切。
    不需要 numpy（纯 python 解 3x3），方便在 PC 上测。
    """

    def __init__(self, coeffs=None, rms=None, n=0, source=""):
        self.coeffs = coeffs          # [a,b,c,d,e,f]
        self.rms = rms                # 拟合残差（mm）
        self.n = n                    # 用了几个点
        self.source = source

    # -- 拟合 ----------------------------------------------------------
    @staticmethod
    def fit(points):
        """
        points: [(u, v, x, y), ...]，至少 3 个不共线的点（强烈建议 ≥6 个）。
        返回 (AffineCalib, report_dict)
        """
        n = len(points)
        if n < 3:
            raise ValueError("至少要 3 个标定点，现在只有 %d 个" % n)

        # 法方程： (A^T A) [a,b,c]^T = A^T X ，A 的每行是 [u, v, 1]
        suu = sum(p[0] * p[0] for p in points)
        suv = sum(p[0] * p[1] for p in points)
        su = sum(p[0] for p in points)
        svv = sum(p[1] * p[1] for p in points)
        sv = sum(p[1] for p in points)
        sxu = sum(p[0] * p[2] for p in points)
        sxv = sum(p[1] * p[2] for p in points)
        sx = sum(p[2] for p in points)
        syu = sum(p[0] * p[3] for p in points)
        syv = sum(p[1] * p[3] for p in points)
        sy = sum(p[3] for p in points)
        M = [[suu, suv, su],
             [suv, svv, sv],
             [su, sv, float(n)]]

        def solve3(M, rhs):
            """Cramer 法则解 3x3。返回 None 表示奇异（点共线）。"""
            def det3(m):
                return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
            D = det3(M)
            if abs(D) < 1e-12:
                return None
            out = []
            for col in range(3):
                m = [row[:] for row in M]
                for r in range(3):
                    m[r][col] = rhs[r]
                out.append(det3(m) / D)
            return out

        cx = solve3(M, [sxu, sxv, sx])
        cy = solve3(M, [syu, syv, sy])
        if cx is None or cy is None:
            raise ValueError("标定点共线/重合，拟合不出来 —— 请让点在画面上散开（例如 3x3 网格）")

        coeffs = [cx[0], cx[1], cx[2], cy[0], cy[1], cy[2]]
        cal = AffineCalib(coeffs, n=n)
        # 残差
        errs = []
        for (u, v, x, y) in points:
            px, py = cal.map(u, v)
            errs.append(math.hypot(px - x, py - y))
        cal.rms = math.sqrt(sum(e * e for e in errs) / n)
        report = {
            "n": n,
            "rms_mm": round(cal.rms, 2),
            "max_mm": round(max(errs), 2),
            "per_point_mm": [round(e, 2) for e in errs],
            "coeffs": [round(c, 6) for c in coeffs],
        }
        return cal, report

    # -- 使用 ----------------------------------------------------------
    def map(self, u, v):
        c = self.coeffs
        return (c[0] * u + c[1] * v + c[2],
                c[3] * u + c[4] * v + c[5])

    def to_dict(self):
        return {"type": "affine", "coeffs": self.coeffs, "rms_mm": self.rms,
                "n": self.n, "source": self.source}

    @staticmethod
    def from_dict(d):
        return AffineCalib(list(d["coeffs"]), d.get("rms_mm"), d.get("n", 0),
                           d.get("source", ""))


# ----------------------------------------------------------------------
# 三、范围检查（拒绝，而不是钳位）
# ----------------------------------------------------------------------

DEFAULT_LIMITS = {
    "x_min": -140.0, "x_max": 140.0,      # mm，比离线复算的可达边缘内收 10mm
    "y_min": 80.0, "y_max": 300.0,
    "z_min": 0.0, "z_max": 250.0,
}


def check_pixel(u, v, roi, margin=0):
    """检测框中心是否落在标定框内。返回 (ok, reason)。"""
    x0, y0, x1, y1 = roi
    x0 -= margin; y0 -= margin; x1 += margin; y1 += margin
    if u < x0 or u > x1 or v < y0 or v > y1:
        return False, ("目标中心 (%.0f,%.0f) 在标定框 [%.0f,%.0f]x[%.0f,%.0f] 之外 "
                       "—— 按厂商逻辑会被钳位到边界，这里直接拒绝" % (u, v, x0, y0, x1, y1))
    return True, "OK"


def check_arm(x, y, z, limits=None):
    """机械臂目标点是否在允许范围内。返回 (ok, reason)。"""
    L = dict(DEFAULT_LIMITS)
    if limits:
        L.update(limits)
    if not (L["x_min"] <= x <= L["x_max"]):
        return False, "X=%.1f mm 超出 [%.0f,%.0f]" % (x, L["x_min"], L["x_max"])
    if not (L["y_min"] <= y <= L["y_max"]):
        return False, "Y=%.1f mm 超出 [%.0f,%.0f]" % (y, L["y_min"], L["y_max"])
    if not (L["z_min"] <= z <= L["z_max"]):
        return False, "Z=%.1f mm 超出 [%.0f,%.0f]" % (z, L["z_min"], L["z_max"])
    return True, "OK"


# ----------------------------------------------------------------------
# 三之二、决策链：选目标 → 换算 → 限幅 → 逆解
#         （放在这里而不是节点里，是为了能在 PC 上离线单测整条决策）
# ----------------------------------------------------------------------

def select_target(detections, mode="center", img_w=640, img_h=480,
                  target_class="", min_area=0.0):
    """
    detections: [{class_name, confidence, x1, y1, x2, y2}, ...]（直接来自 /yolov8_detections）

    mode:
        "center"  离画面中心最近（默认 —— 车停到位后目标一般就在中间）
        "largest" 面积最大（离得最近的东西）
        "arm"     离机械臂原点(画面下方正中偏右)最近

    target_class: 空 = 任意类；非空 = 只认这一类（逗号分隔可给多个）
    返回 (det_or_None, reason)
    """
    if not detections:
        return None, "还没有任何检测结果"

    allow = set(c.strip() for c in target_class.split(",") if c.strip())
    cands = []
    for d in detections:
        if allow and d.get("class_name") not in allow:
            continue
        w = abs(d["x2"] - d["x1"])
        h = abs(d["y2"] - d["y1"])
        if w * h < min_area:
            continue
        cands.append((d, (d["x1"] + d["x2"]) / 2.0, (d["y1"] + d["y2"]) / 2.0, w * h))

    if not cands:
        if allow:
            return None, "画面里没有 %s 这几类（有检测，但类别对不上）" % ",".join(sorted(allow))
        return None, "画面里没有满足 min_area 的目标"

    if mode == "largest":
        det, u, v, _ = max(cands, key=lambda t: t[3])
    elif mode == "arm":
        # 机械臂大致在画面下方偏右（厂商 bbox 标定框中心约 (339,240)，
        # 而抓取落点由 Y 越大越远，画面下方=近处）
        det, u, v, _ = min(cands, key=lambda t: math.hypot(t[1] - img_w * 0.55,
                                                           t[2] - img_h * 0.85))
    else:
        det, u, v, _ = min(cands, key=lambda t: math.hypot(t[1] - img_w / 2.0,
                                                           t[2] - img_h / 2.0))
    return {"det": det, "u": u, "v": v, "n_candidates": len(cands)}, "OK"


def plan_grasp(u, v, calib, kin, params=None):
    """
    像素点 → 一次抓取的完整计划。

    params 可用键（都有默认值）：
        z            抓取高度 mm（默认 10，与厂商一致）
        offset_x     机械臂 X 微调 mm（默认 0）
        offset_y     机械臂 Y 微调 mm（默认 0）
        prefer_alpha 期望爪面角（None = 厂商行为：取最负可行值）
        limits       dict，覆盖 DEFAULT_LIMITS
        roi          (x0,y0,x1,y1) 像素范围；None = 用 calib 自带的
        roi_margin   像素容差（默认 0）

    返回 dict: ok / reason / u / v / x / y / z / alpha / pwm / stage
    """
    p = {"z": 10.0, "offset_x": 0.0, "offset_y": 0.0,
         "prefer_alpha": None, "limits": None, "roi": None, "roi_margin": 0}
    if params:
        p.update(params)

    out = {"ok": False, "reason": "", "stage": "", "u": u, "v": v,
           "x": None, "y": None, "z": p["z"], "alpha": None, "pwm": None}

    # ① 像素范围（拒绝，不钳位）
    roi = p["roi"]
    if roi is None and calib is not None and hasattr(calib, "roi"):
        roi = calib.roi()
    if roi is not None:
        ok, why = check_pixel(u, v, roi, p["roi_margin"])
        if not ok:
            out.update(stage="pixel_range", reason=why)
            return out

    # ② 像素 → 机械臂
    if calib is None:
        out.update(stage="calib", reason="没有可用标定")
        return out
    x, y = calib.map(u, v)
    x += p["offset_x"]
    y += p["offset_y"]
    out.update(x=x, y=y)

    # ③ 限幅
    ok, why = check_arm(x, y, p["z"], p["limits"])
    if not ok:
        out.update(stage="arm_range", reason=why)
        return out

    # ④ 逆解
    r = kin.plan(x, y, p["z"], prefer_alpha=p["prefer_alpha"])
    out.update(alpha=r["alpha"], pwm=r["pwm"])
    if not r["ok"]:
        out.update(stage="ik", reason=r["reason"])
        return out

    out.update(ok=True, reason="OK", stage="done")
    return out


# ----------------------------------------------------------------------
# 四、标定文件读写
# ----------------------------------------------------------------------

def load_calib(path):
    """读标定 json。不存在/坏掉就返回 None（调用方回落到 VendorInterp）。"""
    if not path or not os.path.exists(path):
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
        if d.get("type") == "affine" and d.get("coeffs"):
            return AffineCalib.from_dict(d)
    except Exception:
        return None
    return None


def save_calib(path, cal):
    d = cal.to_dict()
    d["created"] = __import__("time").strftime("%Y-%m-%d %H:%M:%S")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    os.replace(tmp, path)
    return d


# ----------------------------------------------------------------------
# 五、舵机指令拼装（和 z_kinematics.py 的格式一字不改）
# ----------------------------------------------------------------------

def fmt_multi(pwm, t_ms):
    """多舵机同步动作：{#000P1500T1000!#001P1500T1000!...}"""
    s = "{"
    for i, p in enumerate(pwm):
        s += "#%03dP%04dT%04d!" % (i, int(p), int(t_ms))
    return s + "}"


def fmt_multi_map(pid_pwm, t_ms):
    """
    指定 id 的多舵机同步动作，id 可以不连续。
    用于"0~3 号按 IK、4 号跟 0 号（腕部跟着云台转）、5 号夹爪"这种混合下发。
    pid_pwm: {id: pwm}
    """
    s = "{"
    for sid in sorted(pid_pwm):
        s += "#%03dP%04dT%04d!" % (int(sid), int(pid_pwm[sid]), int(t_ms))
    return s + "}"


def fmt_one(sid, pwm, t_ms):
    return "#%03dP%04dT%04d!" % (int(sid), int(pwm), int(t_ms))


def fmt_read(sid):
    return "#%03dPRAD!" % int(sid)


# ----------------------------------------------------------------------
# 六、自测
# ----------------------------------------------------------------------

def _selftest():
    fails = []

    def chk(name, cond, detail=""):
        if cond:
            print("  [PASS] %s" % name)
        else:
            print("  [FAIL] %s   %s" % (name, detail))
            fails.append(name)

    print("1) 逆运动学：与离线复算的结果逐一对齐（回归测试）")
    k = Kinematics(100, 105, 75, 185)
    cases = [
        # (x, y, z, 期望 pwm, 期望 alpha)
        ((0, 200, 10), [1500, 1159, 1741, 867], -74),
        ((-88, 190, 10), [1684, 1137, 1707, 869], -72),
        ((110, 280, 10), [1341, 995, 1640, 1137], -46),
        ((11, 235, 10), [1480, 1074, 1612, 882], -66),
    ]
    for (x, y, z), want_pwm, want_alpha in cases:
        r = k.plan(x, y, z)
        chk("目标(%.0f,%.0f,%d)" % (x, y, z),
            r["ok"] and r["pwm"] == want_pwm and r["alpha"] == want_alpha,
            "得到 pwm=%s alpha=%s（期望 %s / %s）" % (r["pwm"], r["alpha"], want_pwm, want_alpha))

    print("2) 逆运动学：L0=110 的差异（说明为什么必须量真实臂长）")
    k2 = Kinematics(110, 105, 75, 185)
    r100 = k.plan(0, 200, 10)
    r110 = k2.plan(0, 200, 10)
    d = r110["pwm"][1] - r100["pwm"][1]
    chk("L0 差 10mm → 1 号舵机差 36 tick", abs(d) == 36, "实际差 %d" % d)

    print("3) 越界一律拒绝（不静默钳位）")
    chk("x=200 超上限被拒", not check_arm(200, 200, 10)[0])
    chk("y=50 低于下限被拒", not check_arm(0, 50, 10)[0])
    chk("x=0,y=0,z=1000 被拒（原码会 ZeroDivisionError）",
        not k.plan(0, 0, 1000)["ok"])

    print("4) 厂商插值与 grab.py 行为一致（在标定框角上应正好命中）")
    vi = VendorInterp()
    chk("左下角", vi.map(187, 362) == (-88, 170), str(vi.map(187, 362)))
    chk("右上角", vi.map(492, 119) == (110, 260), str(vi.map(492, 119)))
    chk("框外被钳位（复刻厂商行为）", vi.map(0, 0) == (-88, 260), str(vi.map(0, 0)))
    chk("像素范围检查能拦住框外", not check_pixel(0, 0, vi.roi())[0])

    print("5) 仿射拟合：用已知变换造 6 个点，看能不能还原")
    true = [0.649, -0.10, -200.0, 0.05, -0.370, 300.0]

    def fwd(u, v):
        return (true[0] * u + true[1] * v + true[2],
                true[3] * u + true[4] * v + true[5])

    pts = []
    for u, v in [(187, 362), (492, 119), (340, 240), (250, 300), (430, 180), (300, 150)]:
        x, y = fwd(u, v)
        pts.append((u, v, x, y))
    cal, rep = AffineCalib.fit(pts)
    chk("残差 RMS < 0.01 mm", rep["rms_mm"] < 0.01, "rms=%s" % rep["rms_mm"])
    u, v = 400, 200
    px, py = cal.map(u, v)
    ex, ey = fwd(u, v)
    chk("映射误差 < 0.01 mm", abs(px - ex) < 0.01 and abs(py - ey) < 0.01,
        "得(%.4f,%.4f) 期望(%.4f,%.4f)" % (px, py, ex, ey))

    print("6) 仿射拟合：共线的点必须报错（不能悄悄给个烂结果）")
    try:
        AffineCalib.fit([(100, 100, 0, 0), (200, 200, 10, 10), (300, 300, 20, 20)])
        chk("共线点被拒", False, "没有抛异常")
    except ValueError:
        chk("共线点被拒", True)

    print("7) 指令格式")
    chk("多舵机", fmt_multi([1500, 1159, 1741, 867], 2000)
        == "{#000P1500T2000!#001P1159T2000!#002P1741T2000!#003P0867T2000!}",
        fmt_multi([1500, 1159, 1741, 867], 2000))
    chk("单舵机", fmt_one(5, 1200, 500) == "#005P1200T0500!", fmt_one(5, 1200, 500))
    chk("读角度", fmt_read(5) == "#005PRAD!", fmt_read(5))
    chk("指定 id 下发（0~4 号，跳过夹爪）",
        fmt_multi_map({0: 1500, 1: 1159, 2: 1741, 3: 867, 4: 1500}, 2000)
        == "{#000P1500T2000!#001P1159T2000!#002P1741T2000!#003P0867T2000!#004P1500T2000!}",
        fmt_multi_map({0: 1500, 1: 1159, 2: 1741, 3: 867, 4: 1500}, 2000))

    print("8) prefer_alpha：默认与厂商一致，指定后选更立/更平的姿态")
    a_default = k.plan(0, 200, 10)
    a_pref = k.plan(0, 200, 10, prefer_alpha=-45)
    chk("默认取最负", a_default["alpha"] == min(
        [a for a in range(-135, 0) if k.solve(0, 200, 10, a)[0] == 0]))
    chk("prefer=-45 与默认不同（说明偏好真的生效）",
        a_pref["alpha"] != a_default["alpha"] or a_default["alpha"] == -45,
        "default=%s pref=%s" % (a_default["alpha"], a_pref["alpha"]))

    print("9) 选目标：中间优先 / 最大优先 / 类别过滤")
    dets = [
        {"class_name": "bottle", "confidence": 0.7, "x1": 20, "y1": 20, "x2": 90, "y2": 90},
        {"class_name": "cup", "confidence": 0.6, "x1": 280, "y1": 200, "x2": 360, "y2": 280},
        {"class_name": "person", "confidence": 0.9, "x1": 500, "y1": 100, "x2": 640, "y2": 460},
    ]
    s = select_target(dets, mode="center")
    chk("center 选中中间那个（cup）", s[1] == "OK" and s[0]["det"]["class_name"] == "cup",
        str(s[0] and s[0]["det"]["class_name"]))
    s = select_target(dets, mode="largest")
    chk("largest 选中面积最大的（person）", s[1] == "OK" and s[0]["det"]["class_name"] == "person",
        str(s[0] and s[0]["det"]["class_name"]))
    s = select_target(dets, mode="center", target_class="bottle")
    chk("类别过滤 bottle", s[1] == "OK" and s[0]["det"]["class_name"] == "bottle")
    s = select_target(dets, mode="center", target_class="car")
    chk("类别过滤不到时明确报原因（不是静默 None）",
        s[0] is None and "类别对不上" in s[1], s[1])
    s = select_target([], mode="center")
    chk("空列表也有原因", s[0] is None and s[1] != "", s[1])

    print("10) 决策链：像素 -> 限幅 -> 逆解，每一环失败都要能说清是哪一环")
    vi = VendorInterp()
    kk = Kinematics(100, 105, 75, 185)
    # 标定框正中
    r = plan_grasp(339, 240, vi, kk, {"z": 10})
    chk("框内中心点能算出 pwm", r["ok"] and r["pwm"] is not None,
        "%s / %s" % (r["stage"], r["reason"]))
    # 框外
    r = plan_grasp(10, 10, vi, kk, {"z": 10})
    chk("框外被 pixel_range 拦下", not r["ok"] and r["stage"] == "pixel_range",
        "%s / %s" % (r["stage"], r["reason"]))
    # 越过 Y 上限（用大 offset 逼出来）
    r = plan_grasp(492, 119, vi, kk, {"z": 10, "offset_y": 100})
    chk("offset 把 Y 顶出上限 -> arm_range", not r["ok"] and r["stage"] == "arm_range",
        "%s / %s / y=%s" % (r["stage"], r["reason"], r["x"] and round(r["y"], 1)))
    # 正常的角点
    r = plan_grasp(492, 119, vi, kk, {"z": 10})
    chk("右上角规划成功且 pwm 合理",
        r["ok"] and all(500 <= v <= 2500 for v in r["pwm"]), str(r["pwm"]))

    print()
    if fails:
        print("[FAIL] %d 项失败：%s" % (len(fails), ", ".join(fails)))
        return 1
    print("[PASS] 全部通过")
    return 0


def _demo():
    here = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(here, "..", "config", "grasp_calib.json")
    cal = load_calib(cfg)
    print("标定文件：%s" % os.path.normpath(cfg))
    if cal is None:
        print("  （没有/读不出 —— 现在会回落到厂商的 4 点插值）")
        cal = VendorInterp()
        print("  厂商框：%s" % (cal.roi(),))
    else:
        print("  仿射系数 %s  rms=%.2fmm  n=%d" % (
            [round(c, 4) for c in cal.coeffs], cal.rms or -1, cal.n))
    k = Kinematics(100, 105, 75, 185)
    print()
    print("  %-12s %-18s %-8s %-8s %s" % ("像素", "机械臂(mm)", "alpha", "z", "PWM"))
    for (u, v) in [(320, 240), (187, 362), (492, 119), (400, 300)]:
        if hasattr(cal, "roi"):
            ok, why = check_pixel(u, v, cal.roi())
        else:
            ok, why = True, "OK"
        x, y = cal.map(u, v)
        r = k.plan(x, y, 10)
        print("  (%3d,%3d)  (%7.1f,%7.1f)  %-8s %-8s %s"
              % (u, v, x, y, r["alpha"], 10, r["pwm"] if r["ok"] else "不可达:" + r["reason"]))
        if not ok:
            print("              ⚠ %s" % why)


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    elif "--demo" in sys.argv:
        _demo()
    else:
        print(__doc__)
        print("可用：--selftest / --demo")
