#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""a.py —— 机械臂手打调试台（命令尽量短，不需要 ROS）

车上常用（先 cd /media/abot/999/abot）：

    python3 a.py g 0 200 150    抬到 15cm
    python3 a.py g 0 200 10     降到桌面
    python3 a.py c              合爪
    python3 a.py o              张爪
    python3 a.py home           回中位（6 个舵机一起，慢）
    python3 a.py probe          只读 6 个舵机（不动）

其他：

    python3 a.py p 5 1800       5 号直接给 1800
    python3 a.py n 5 60         5 号 +60（基准是 HOME，不累加）
    python3 a.py grip           夹爪扫描 1200~2400，每步都问
    python3 a.py s '#005P1500T1000!'   原样发一条

坐标：x 右为正、y 前为正、z = 爪尖离底座面的高度（mm）。
      (0, 200) = 臂正中线、正前方 20cm。

要改的常数就在下面这几行，nano 改完直接生效。
"""
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("没装 pyserial：先跑 python3 -c 'import serial' 确认")

# ======== 可以改的常数 ========
L = (100, 105, 75, 185)   # 臂长 l0 l1 l2 l3（mm）。量完真实 l0 就改第一个数
MOVE_MS = 2000            # 一个 g 姿态走多久
GRIP_OPEN = 1200          # o：张开
GRIP_CLOSE = 1800         # c：夹紧
BAUD = 115200
LOG_DIR = "/media/abot/999/abot/_运行日志"
# ==============================

HOME = {0: 1500, 1: 1500, 2: 1800, 3: 700, 4: 1500, 5: 1500}
CUR = dict(HOME)
LOG_FILE = os.path.join(LOG_DIR, "arm-%s.log" % time.strftime("%Y%m%d"))

# 逆运动学用 grasp_math（就在本工作区里，已经过 31 项离线自测）
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "src", "yolov8_ros", "script"))
try:
    import grasp_math
    _IMPORT_ERR = None
except Exception as exc:
    grasp_math = None
    _IMPORT_ERR = exc


def log(msg):
    print(msg)
    try:
        if not os.path.isdir(LOG_DIR):
            os.makedirs(LOG_DIR)
        with open(LOG_FILE, "a", encoding="utf-8") as f:
            f.write("%s  %s\n" % (time.strftime("%H:%M:%S"), msg))
    except Exception:
        pass


def real(p):
    try:
        if os.path.exists(p):
            return os.path.realpath(p)
    except Exception:
        pass
    return p


def find_port(explicit=None):
    if explicit:
        return explicit
    env = os.environ.get("ABOT_ARM_PORT")
    if env:
        return env
    if os.path.exists("/dev/arm"):
        return "/dev/arm"
    used = set([real("/dev/abot_driver"), real("/dev/rplidar")])
    cands = [d for d in sorted(glob.glob("/dev/ttyUSB*")) if real(d) not in used]
    if len(cands) == 1:
        return cands[0]
    return None


def confirm(what):
    log("将要：%s" % what)
    try:
        return input("输 YES: ").strip().upper() == "YES"
    except (EOFError, KeyboardInterrupt):
        return False


def send(s, text, wait):
    log("  -> %s" % text)
    s.write(text.encode())
    if wait > 0:
        time.sleep(wait)


def absorb(s, seconds):
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        buf += s.read(64)
    return buf


def one(s, sid, pwm, t=800):
    pwm = max(500, min(2500, int(pwm)))
    send(s, "#%03dP%04dT%04d!" % (sid, pwm, t), t / 1000.0 + 0.2)
    CUR[sid] = pwm
    return pwm


# ---------------- 命令 ----------------

def cmd_probe(s):
    ok = 0
    for i in range(6):
        s.reset_input_buffer()
        send(s, "#%03dPRAD!" % i, 0)
        b = absorb(s, 0.5)
        if b:
            ok += 1
        log("  %d 号 <- %r" % (i, b[:40]))
    log("回包 %d/6" % ok)
    return 0


def cmd_home(s, _a):
    t = 3000
    if not confirm("6 个舵机回中位（%d ms）" % t):
        return 2
    body = "{" + "".join("#%03dP%04dT%04d!" % (i, HOME[i], t) for i in range(6)) + "}"
    send(s, body, t / 1000.0 + 0.5)
    CUR.update(HOME)
    log("已回中位")
    return 0


def cmd_g(s, a):
    if grasp_math is None:
        log("[!!] 载入 grasp_math 失败：%s" % _IMPORT_ERR)
        log("     确认 src/yolov8_ros/script/grasp_math.py 还在")
        return 1
    try:
        x, y, z = float(a[0]), float(a[1]), float(a[2])
    except (IndexError, ValueError):
        log("用法：g x y z     例：g 0 200 150")
        return 2
    k = grasp_math.Kinematics(*L)
    r = k.plan(x, y, z)
    if not r["ok"]:
        log("[!!] (%.0f,%.0f,%.0f) 不可达：%s" % (x, y, z, r["reason"]))
        return 1
    p = r["pwm"]
    body = grasp_math.fmt_multi_map({0: p[0], 1: p[1], 2: p[2], 3: p[3], 4: p[0]}, MOVE_MS)
    if not confirm("走去 (%.0f,%.0f,%.0f) alpha=%d" % (x, y, z, r["alpha"])):
        return 2
    send(s, body, MOVE_MS / 1000.0 + 0.5)
    log("到位")
    return 0


def cmd_o(s, _a):
    log("张爪 -> %d" % GRIP_OPEN)
    one(s, 5, GRIP_OPEN, 1500)
    return 0


def cmd_c(s, _a):
    log("合爪 -> %d" % GRIP_CLOSE)
    one(s, 5, GRIP_CLOSE, 1500)
    return 0


def cmd_p(s, a):
    try:
        sid, pwm = int(a[0]), int(a[1])
    except (IndexError, ValueError):
        log("用法：p 5 1800")
        return 2
    if sid < 0 or sid > 5 or pwm < 500 or pwm > 2500:
        log("p <0-5> <500-2500>")
        return 2
    if not confirm("%d 号 -> %d" % (sid, pwm)):
        return 2
    log("已发")
    one(s, sid, pwm)
    return 0


def cmd_n(s, a):
    try:
        sid, d = int(a[0]), int(a[1])
    except (IndexError, ValueError):
        log("用法：n 5 60")
        return 2
    if sid < 0 or sid > 5:
        log("舵机号 0~5")
        return 2
    if abs(d) > 120:
        log("单次最多 120 tick")
        return 2
    v = max(500, min(2500, CUR.get(sid, HOME[sid]) + d))
    if not confirm("%d 号 %d -> %d" % (sid, CUR.get(sid, HOME[sid]), v)):
        return 2
    log("已发")
    one(s, sid, v)
    return 0


def cmd_grip(s, _a):
    for v in (1200, 1400, 1600, 1800, 2000, 2200, 2400):
        if not confirm("5 号 -> %d" % v):
            log("停在 %d" % CUR.get(5, HOME[5]))
            return 2
        send(s, "#005P%04dT1500!" % v, 1.9)
        CUR[5] = v
    log("扫描完，记下 张到最大 / 夹住木块 两个值")
    return 0


def cmd_s(s, a):
    text = " ".join(a)
    if not text:
        log("用法：s '#005P1500T1000!'")
        return 2
    if not confirm("原样发 %s" % text):
        return 2
    send(s, text, 1.0)
    log("   <- %r" % absorb(s, 0.5)[:60])
    return 0


CMDS = {
    "probe": (cmd_probe, 0),
    "home": (cmd_home, 0),
    "g": (cmd_g, 3),
    "o": (cmd_o, 0),
    "c": (cmd_c, 0),
    "p": (cmd_p, 2),
    "n": (cmd_n, 2),
    "nudge": (cmd_n, 2),
    "grip": (cmd_grip, 0),
    "s": (cmd_s, 1),
    "raw": (cmd_s, 1),
}

USAGE = """用法（cd /media/abot/999/abot 之后）：
  python3 a.py g 0 200 150   抬到 15cm
  python3 a.py g 0 200 10    降到桌面
  python3 a.py c             合爪
  python3 a.py o             张爪
  python3 a.py home          回中位
  python3 a.py probe         只读 6 个舵机
  python3 a.py p 5 1800      5 号直接给 1800
  python3 a.py n 5 60        5 号 +60
  python3 a.py grip          夹爪扫描
  python3 a.py s '#005P1500T1000!'   原样发
"""


def main(argv):
    if not argv or argv[0] in ("-h", "--help", "help"):
        print(USAGE)
        print("当前串口：%s" % (" ".join(sorted(glob.glob("/dev/ttyUSB*"))) or "（没有）"))
        print("自动判定：%s" % (find_port() or "（没找到，用 ABOT_ARM_PORT 指定）"))
        return 0

    port = None
    if "--port" in argv:
        i = argv.index("--port")
        if i + 1 >= len(argv):
            log("--port 后面要跟设备名")
            return 2
        port = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]

    sub = argv[0]
    args = argv[1:]
    if sub not in CMDS:
        log("不认识的命令：%s" % sub)
        print(USAGE)
        return 2
    fn, nargs = CMDS[sub]
    if len(args) < nargs:
        log("%s 需要 %d 个参数" % (sub, nargs))
        print(USAGE)
        return 2

    port = find_port(port)
    if not port or not os.path.exists(port):
        log("[!!] 找不到机械臂串口（%s）" % (port or "自动判定失败"))
        log("     ls -l /dev/arm /dev/ttyUSB*  然后 export ABOT_ARM_PORT=/dev/ttyUSBx")
        return 3

    log("端口 %s" % port)
    try:
        s = serial.Serial(port, BAUD, timeout=0.2)
        s.reset_input_buffer()
    except Exception as e:
        log("[!!] 打不开 %s：%s" % (port, e))
        log("     Permission denied -> sudo python3 a.py ...；其它 -> 换 --port")
        return 3
    log("[OK] 已打开")
    try:
        return fn(s, args)
    finally:
        try:
            s.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\n中断（舵机可能还在走完那一条）")
        sys.exit(130)
