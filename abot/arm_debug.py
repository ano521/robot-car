#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arm_debug.py —— ABOT 机械臂上机调试（不需要 ROS，不需要 roscore）

用法（在车上，先跑一次 ./killall.sh）：
    python3 arm_debug.py probe             # 找口 + 只读 6 个舵机（不动）
    python3 arm_debug.py home              # 慢速回中位（要输 YES）
    python3 arm_debug.py nudge 5 60        # 5 号在当前位置上 +60 tick（约 8 度，要输 YES）
    python3 arm_debug.py p 5 1700          # 5 号直接发 1700（要输 YES）
    python3 arm_debug.py grip              # 夹爪扫描 1200~2400，每步都问你
    python3 arm_debug.py raw '#005P1500T1000!'    # 原样发一条（要输 YES）

  指定/覆盖端口：--port /dev/ttyUSB2
  自动找口顺序：--port  >  环境变量 ABOT_ARM_PORT  >  /dev/arm  >  排除底盘/雷达后的那个 ttyUSB

舵机号：0=底座旋转  1/2/3=大臂/小臂/腕摆  4=腕转  5=夹爪
PWM：1500=中位，±2000 对应 270 度（1 度约 7.4 tick）

⚠️ Ctrl-C 只结束本程序；舵机收到 T 毫秒的指令后会自己走完。
   要立刻停，切电源（或马上发一条回当前位的指令）。

所有输出会同时追加到 /media/abot/999/abot/_运行日志/arm-YYYYMMDD.log
"""
import glob
import os
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("没装 pyserial。用 python3 -c 'import serial' 确认一下")

HOME = {0: 1500, 1: 1500, 2: 1800, 3: 700, 4: 1500, 5: 1500}
CUR = dict(HOME)
BAUD = 115200
LOG_DIR = "/media/abot/999/abot/_运行日志"
LOG_FILE = os.path.join(LOG_DIR, "arm-%s.log" % time.strftime("%Y%m%d"))


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


def find_port(explicit):
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
    if cands:
        log("[!!] 有 %d 个候选口，不敢乱猜：%s" % (len(cands), " ".join(cands)))
    return None


def confirm(what):
    log("将要执行：%s" % what)
    try:
        a = input("确认请输 YES（其它任意键取消）: ").strip()
    except (EOFError, KeyboardInterrupt):
        return False
    return a == "YES"


def open_port(port):
    s = serial.Serial(port, BAUD, timeout=0.2)
    s.reset_input_buffer()
    return s


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


def cmd_probe(s):
    log("只读 6 个舵机（不动）")
    ok = 0
    for i in range(6):
        s.reset_input_buffer()
        send(s, "#%03dPRAD!" % i, 0)
        b = absorb(s, 0.5)
        if b:
            ok += 1
        log("   %d 号 <- %r" % (i, b[:48]))
    log("有回包的舵机数：%d / 6" % ok)
    if ok == 0:
        log("[~~] 一个都没回包。可能是：舵机板不主动回 RAD / 这个口不是机械臂 / 波特率不对")
        return 1
    return 0


def cmd_home(s):
    t = 3000
    if not confirm("6 个舵机一起慢速回中位（%d ms）" % t):
        return 2
    body = "{" + "".join("#%03dP%04dT%04d!" % (i, HOME[i], t) for i in range(6)) + "}"
    send(s, body, t / 1000.0 + 0.5)
    CUR.update(HOME)
    log("已回中位")
    return 0


def move_one(s, sid, pwm, t=800):
    pwm = max(500, min(2500, int(pwm)))
    send(s, "#%03dP%04dT%04d!" % (sid, pwm, t), t / 1000.0 + 0.2)
    CUR[sid] = pwm
    return pwm


def cmd_nudge(s, sid, delta):
    sid = int(sid)
    delta = int(delta)
    if sid < 0 or sid > 5:
        log("[!!] 舵机号必须是 0~5")
        return 2
    if abs(delta) > 120:
        log("[!!] 单次最多 120 tick（约 16 度），先小步走")
        return 2
    base = CUR.get(sid, HOME[sid])
    pwm = max(500, min(2500, base + delta))
    if not confirm("%d 号：%d -> %d（delta %+d）" % (sid, base, pwm, delta)):
        return 2
    move_one(s, sid, pwm)
    log("已发，现在 %d 号 = %d" % (sid, pwm))
    return 0


def cmd_p(s, sid, pwm):
    sid = int(sid)
    pwm = int(pwm)
    if sid < 0 or sid > 5 or pwm < 500 or pwm > 2500:
        log("[!!] 用法：p <0~5> <500~2500>")
        return 2
    if not confirm("%d 号 -> %d" % (sid, pwm)):
        return 2
    move_one(s, sid, pwm)
    log("已发，现在 %d 号 = %d" % (sid, pwm))
    return 0


def cmd_grip(s):
    log("夹爪扫描：1200 -> 2400，步长 200。记下【张到最大】和【夹住木块】两个值")
    for pwm in (1200, 1400, 1600, 1800, 2000, 2200, 2400):
        if not confirm("5 号 -> %d（停 1.5 秒）" % pwm):
            log("停止扫描，最后一个是 %d" % CUR.get(5, HOME[5]))
            return 2
        send(s, "#005P%04dT1500!" % pwm, 1.9)
        CUR[5] = pwm
    log("扫描完。把上面观察到的两个数填进 grasp_params.yaml")
    return 0


def cmd_raw(s, text):
    if not confirm("原样发送：%s" % text):
        return 2
    send(s, text, 1.0)
    log("   <- %r" % absorb(s, 0.5)[:80])
    return 0


def usage():
    print(__doc__)
    print("当前能找到的串口：%s" % (" ".join(sorted(glob.glob("/dev/ttyUSB*"))) or "（一个都没有）"))
    print("自动判定为：%s" % (find_port(None) or "（没找到，请用 --port 指定）"))


def main(argv):
    if not argv or argv[0] in ("-h", "--help", "help"):
        usage()
        return 0

    explicit = None
    if "--port" in argv:
        i = argv.index("--port")
        if i + 1 >= len(argv):
            log("[!!] --port 后面要跟设备名")
            return 2
        explicit = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]

    sub = argv[0]
    args = argv[1:]

    need = {"nudge": 2, "p": 2, "raw": 1}
    if sub in need and len(args) < need[sub]:
        usage()
        return 2

    port = find_port(explicit)
    if not port:
        log("[!!] 找不到机械臂串口。")
        usage()
        return 3
    if not os.path.exists(port):
        log("[!!] %s 不存在（悬空软链接或没插好）" % port)
        log("     跑 ./7-grasp-check.sh 看看 /dev/arm /dev/ttyUSB*")
        return 3

    log("=== arm_debug：端口 %s @ %d ===" % (port, BAUD))
    try:
        s = open_port(port)
    except Exception as e:
        log("[!!] 打不开 %s：%s" % (port, e))
        log("     Permission denied -> 用 sudo python3 试；其它 -> 换 --port")
        return 3
    log("[OK] 已打开")
    try:
        if sub == "probe":
            return cmd_probe(s)
        if sub == "home":
            return cmd_home(s)
        if sub == "nudge":
            return cmd_nudge(s, args[0], args[1])
        if sub == "p":
            return cmd_p(s, args[0], args[1])
        if sub == "grip":
            return cmd_grip(s)
        if sub == "raw":
            return cmd_raw(s, " ".join(args))
        log("[!!] 未知命令：%s" % sub)
        usage()
        return 2
    finally:
        try:
            s.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        print("\n已中断（注意：舵机可能还在走完那一条指令）")
        sys.exit(130)
