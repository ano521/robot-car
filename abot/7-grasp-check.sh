#!/bin/bash
# ============================================================
#  抓取链路 · 只读体检（第 7 步）
#
#  为什么需要它：
#    抓取链路和导航链路是两条完全不同的硬件。导航那条（底盘/雷达）已经有
#    0-check.sh 管着了；抓取这条用的是【机械臂串口 /dev/arm】和【第二台相机
#    /dev/video2】，而这两个设备 0-check.sh、killall.sh 都没覆盖 ——
#    全仓库 grep，只有 WCXC_UartServer.py 里写了 /dev/arm，文档里从没出现过它。
#
#  本脚本【只读】：不写串口、不发舵机指令、不改参数、不动机械臂。
#    （唯一"碰"串口的动作是 --probe 打开端口读 2 秒，默认关闭且需显式参数）
#
#  用法：
#     cd /media/abot/999/abot
#     ./7-grasp-check.sh            # 只读体检（推荐，第一次就跑这个）
#     ./7-grasp-check.sh --probe    # 额外：真打开 /dev/arm 读 2 秒，看有没有回包
#                                   # ⚠️ 只能在 grab.py/put.py/arm_move.py 都没在跑时用
#
#  配套文档：docs/抓取功能/01-抓取链路通读与断点清单.md
# ============================================================

WS="$(cd "$(dirname "$0")" && pwd)"
PROBE=0
for a in "$@"; do
  case "$a" in
    --probe) PROBE=1 ;;
    -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "未知参数：$a（只支持 --probe）"; exit 2 ;;
  esac
done

# 保证 rospack / rosmsg / 工作区消息类型可用。
if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  . /opt/ros/noetic/setup.bash
fi
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1091
  . "$WS/devel/setup.bash"
fi

ok()   { echo "  [OK]   $1"; }
no()   { echo "  [!!]   $1"; }
warn() { echo "  [~~]   $1"; }
hr()   { echo; echo "------------------------------------------------------------"; }

# 找 fuser：普通用户 PATH 里没有 /usr/sbin（2026-10-12 踩过）
FUSER=""
for c in fuser /usr/sbin/fuser /sbin/fuser /usr/bin/fuser; do
  if command -v "$c" >/dev/null 2>&1; then FUSER="$c"; break; fi
done

echo "============================================================"
echo " ABOT 抓取链路只读体检    $(date '+%F %T')"
echo " 工作区: $WS"
echo "============================================================"

# ------------------------------------------------------------------
hr; echo " 1/7  串口设备总览（三个 USB 串口芯片 -> 三个设备节点）"; hr
echo "  /dev/ttyUSB*："
# 注意：不能写成 `ls ... | sed ... || no ...` —— 管道取的是【最后一个命令】的退出码，
# sed 永远成功，于是 [!!] 永远不会触发。要先判 ls 本身。
if ls /dev/ttyUSB* >/dev/null 2>&1; then
  ls -l /dev/ttyUSB* | sed 's/^/    /'
else
  no "一个 /dev/ttyUSB* 都没有（C7 那一族：USB 掉线没重枚举）"
fi
echo
echo "  驱动绑定（空 = 驱动没绑上）："
USBSER=$(ls -A /sys/bus/usb-serial/devices/ 2>/dev/null)
if [ -n "$USBSER" ]; then
  echo "$USBSER" | sed 's/^/    /'
else
  no "读不到 /sys/bus/usb-serial/devices/ 或它是空的（驱动没绑上）"
fi
echo
echo "  三个别名逐个判（★ 必须先测 -e 再测 -L：悬空软链接也会满足 -L）:"
for d in /dev/abot_driver /dev/rplidar /dev/arm; do
  if [ -e "$d" ]; then
    ok "$d -> $(readlink -f "$d" 2>/dev/null)（真实存在）"
  elif [ -L "$d" ]; then
    no "$d -> $(readlink "$d") 是【悬空软链接】，目标不存在（重插 USB 或重启）"
  else
    no "$d 不存在"
  fi
done

# /dev/arm 是抓取链路的命门：WCXC_UartServer.py 第 19 行 port='/dev/arm'，
# 没有它 grab.py / put.py / keyboard_grasp.py / arm_move.py 全都起不来。
if [ -e /dev/arm ]; then
  ok "抓取链路的机械臂串口在位"
else
  no "★ 机械臂串口 /dev/arm 不在 —— 抓取链路 100% 起不来（详见第 2 节）"
fi

echo
echo "  相机采集节点："
for v in /dev/video0 /dev/video2; do
  if [ -e "$v" ]; then
    ok "$v 存在"
  else
    no "$v 不存在"
  fi
done

# ------------------------------------------------------------------
hr; echo " 2/7  /dev/arm 详查（0-check.sh / killall.sh 都没覆盖的盲区）"; hr
if [ -e /dev/arm ]; then
  ls -l /dev/arm | sed 's/^/    /'
  echo
  echo "  它到底是哪个 USB 串口芯片（用来判断是不是 ttyUSB2）："
  if command -v udevadm >/dev/null 2>&1; then
    udevadm info -q property -n /dev/arm 2>/dev/null \
      | grep -E '^(DEVPATH|ID_VENDOR|ID_MODEL|ID_SERIAL_SHORT|ID_PATH|ID_USB_INTERFACE_NUM)=' \
      | sed 's/^/    /' || warn "udevadm 没查到属性"
  else
    warn "没有 udevadm，跳过"
  fi
  echo
  echo "  当前用户能不能开它（2026-10-28 改：按真实权限位判，不再查 dialout 组）:"
  echo "    为什么改：本车 udev 规则把 ttyUSB* 设成 0666，abot 不在 dialout 组也能开，"
  echo "    旧判据只查 groups，2026-10-28 误报了一次，让人白紧张。"
  TARGET=$(readlink -f /dev/arm 2>/dev/null)
  if [ -n "$TARGET" ] && [ -e "$TARGET" ]; then
    MODE=$(stat -c '%a' "$TARGET" 2>/dev/null)
    OWN=$(stat -c '%U:%G' "$TARGET" 2>/dev/null)
    if [ -r "$TARGET" ] && [ -w "$TARGET" ]; then
      ok "$TARGET  $OWN  mode=$MODE  —— 当前用户可读写（不需要 dialout 组）"
    else
      no "$TARGET  $OWN  mode=$MODE  —— 当前用户不可读写"
      warn "只有在这种情况下才需要考虑组权限；加组属于改车内配置，按规矩不改"
    fi
  else
    warn "读不到 /dev/arm 的真实设备节点，跳过权限判定"
  fi
  echo
  echo "  谁正占着 /dev/arm（应当为空；有输出说明抓取节点还在跑）:"
  if [ -n "$FUSER" ]; then
    HELD=$("$FUSER" -v /dev/arm 2>&1)
    if [ -n "$HELD" ]; then echo "$HELD" | sed 's/^/    /'; else echo "    （无）"; fi
  else
    echo "    （没有 fuser，改用 pgrep）"
    PG=$(pgrep -a -f 'grab.py|put.py|arm_move.py|keyboard_grasp' 2>/dev/null)
    if [ -n "$PG" ]; then echo "$PG" | sed 's/^/    /'; else echo "    （无）"; fi
  fi
else
  echo "  /dev/arm 不存在。三条只读命令定位（别改车内文件）："
  echo "    ls -l /dev/ttyUSB* /sys/bus/usb-serial/devices/"
  echo "    sudo dmesg | grep -iE 'ttyUSB|ch341|cp210x|ftdi' | tail -40"
  echo "    ls /usr/lib/udev/rules.d/ | grep -iE 'abot|arm|rplidar|999'   # 别名是谁的规则"
  echo
  echo "  ★ 若确认机械臂在 ttyUSB2 但别名没建：不要改车内 udev（红线）。"
  echo "    代码侧已预留环境变量 ABOT_ARM_PORT 覆盖（见第 6 节说明）"
fi

# ------------------------------------------------------------------
hr; echo " 3/7  Python 依赖（缺一个，节点就是起来即死）"; hr
for m in serial rospy cv2 numpy ultralytics; do
  v=$(python3 -c "import $m;print(getattr($m,'__version__','?'))" 2>/dev/null)
  if [ -n "$v" ]; then ok "python3 -c 'import $m'  ->  $v"
  else no "python3 -c 'import $m' 失败（$m 没装）"; fi
done
echo
echo "  YOLO 权重（multi_yolo.py 里写死的路径，不要改）："
if [ -s /home/abot/yolov8/yolov8n.pt ]; then
  ok "/home/abot/yolov8/yolov8n.pt  $(du -h /home/abot/yolov8/yolov8n.pt | cut -f1)"
else
  no "/home/abot/yolov8/yolov8n.pt 不存在 -> multi_yolo.py 第 13 行直接抛异常"
fi

echo
echo "  模板匹配资源（水果模板链）："
TPL="$WS/src/abot_find/object_task"
TPL_N=0
for i in 1 2 3 4 5 6 7 8 9; do
  if [ -s "$TPL/$i.png" ]; then
    TPL_N=$((TPL_N + 1))
  else
    no "缺模板 $TPL/$i.png"
  fi
done
if [ "$TPL_N" -eq 9 ]; then
  ok "9 张水果模板齐全"
else
  no "水果模板只找到 $TPL_N/9 张"
fi

if rospack find find_object_2d >/dev/null 2>&1; then
  ok "find_object_2d 包可解析"
else
  no "find_object_2d 包不可解析"
fi

for s in ai_locate.py grasp_once.py grasp_math.py grasp_calib.py; do
  if [ -s "$WS/src/yolov8_ros/script/$s" ]; then
    ok "抓取脚本存在：$s"
  else
    no "抓取脚本缺失：$s"
  fi
done

# ------------------------------------------------------------------
hr; echo " 4/7  ROS 侧：相机话题与 YOLO 订阅是否对得上（已知断点 G1）"; hr
if pgrep -x rosmaster >/dev/null 2>&1; then
  # ★ 先覆盖掉旧热点那个死 IP，否则每条 rostopic 都会无限卡住（C5）
  # shellcheck disable=SC1091
  [ -f "$WS/ros-env.sh" ] && . "$WS/ros-env.sh"

  echo "  谁在发图像（关键）："
  for t in /usb_cam2/image_raw /usb_cam/image_raw; do
    info=$(timeout 5 rostopic info "$t" 2>/dev/null)
    if echo "$info" | grep -q 'Publishers:'; then
      if echo "$info" | grep -A3 'Publishers:' | grep -q ' \* /'; then
        ok "$t 有发布者"
      else
        no "$t 没有发布者"
      fi
      echo "$info" | sed 's/^/        /'
    else
      warn "$t 话题不存在"
    fi
  done
  echo
  echo "  YOLO 节点在订阅哪个话题、检测有没有输出（★ G1 的判据）："
  # ★ multi_yolo.py 里 init_node(..., anonymous=True)，真实节点名带 pid 后缀
  #   （/yolov8_ros_node_xxxxx），写死名字会 'node not found'，必须先从 rosnode list 抓。
  YNODE=$(rosnode list 2>/dev/null | grep -m1 'yolov8')
  if [ -n "$YNODE" ]; then
    echo "    节点名：$YNODE（anonymous=true，所以带后缀）"
    timeout 5 rosnode info "$YNODE" 2>/dev/null | sed -n '1,45p' | sed 's/^/    /'
    echo
    echo "  /yolov8_detections 有没有东西（3 秒采样，看 class_name 字段）："
    n=$(timeout 3 rostopic echo -p /yolov8_detections 2>/dev/null | grep -c 'class_name')
    if [ "${n:-0}" -gt 0 ]; then
      ok "/yolov8_detections 3 秒内出了 $n 行"
    else
    no "/yolov8_detections 3 秒内没消息（相机已出图不代表模板/YOLO 已识别到目标）"
    fi
  else
    warn "yolov8 节点没在跑（没起 grasp.sh 时正常）"
  fi
  echo
  echo "  模板识别节点 / 翻译节点："
  FNODE=$(rosnode list 2>/dev/null | grep -m1 'find_object_2d')
  ANODE=$(rosnode list 2>/dev/null | grep -m1 'ai_locate')
  if [ -n "$FNODE" ]; then
    ok "find_object_2d 在跑：$FNODE"
  else
    warn "find_object_2d 没在跑（只测 YOLO 时可忽略）"
  fi
  if [ -n "$ANODE" ]; then
    ok "ai_locate 在跑：$ANODE"
  else
    warn "ai_locate 没在跑（用 ai.sh / ai_grasp.launch 时应存在）"
  fi
  echo
  echo "  模板匹配话题："
  for t in /objects /ai_locate/ids; do
    info=$(timeout 5 rostopic info "$t" 2>/dev/null)
    if echo "$info" | grep -A3 'Publishers:' | grep -q ' \* /'; then
      ok "$t 有发布者"
    else
      no "$t 没有发布者或话题不存在"
    fi
  done
  echo
  echo "  抓取相关参数（当前值）："
  for p in /grab /put /success; do
    echo "    $p = $(timeout 3 rosparam get $p 2>/dev/null || echo '（未设置）')"
  done
  echo
  echo "  代价地图各层 enabled（必须 true；false = D9 复发，路径会穿墙）:"
  for k in local_costmap/obstacle_layer global_costmap/static_layer local_costmap/inflation_layer; do
    v=$(timeout 3 rosparam get "/move_base/$k/enabled" 2>/dev/null)
    if [ "$v" = "True" ] || [ "$v" = "true" ]; then ok "$k = $v"; else no "$k = ${v:-未设置}"; fi
  done
else
  warn "roscore 没在跑 —— 本节跳过（要看话题请先起 grasp.sh 或 3-amcl_test.sh）"
fi

# ------------------------------------------------------------------
hr; echo " 5/7  机械臂串口回读（--probe 才做，默认跳过）"; hr
if [ "$PROBE" != "1" ]; then
  warn "未启用。要真的读一次串口，跑：./7-grasp-check.sh --probe"
  echo "        （前提：grab.py / put.py / arm_move.py / keyboard_grasp.py 都没在跑）"
else
  echo "  [!] 正在打开 /dev/arm @115200 读 2 秒（只读，不发任何舵机指令）"
  python3 - <<'PY'
import os, sys, time
port = os.environ.get('ABOT_ARM_PORT', '/dev/arm')
try:
    import serial
except ImportError:
    print("  [!!] pyserial 没装"); sys.exit(1)
try:
    s = serial.Serial(port=port, baudrate=115200, timeout=0.3)
except Exception as e:
    print("  [!!] 打不开 %s : %s" % (port, e)); sys.exit(1)
buf = b''
t0 = time.time()
while time.time() - t0 < 2.0:
    buf += s.read(256)
s.close()
if buf:
    print("  [OK] 2 秒内收到 %d 字节：%r" % (len(buf), buf[:120]))
    print("       有回包 = 串口通、舵机总线活着")
else:
    print("  [~~] 2 秒内没收到任何字节")
    print("       ① 舵机板不主动上报（正常，发指令才有回包）")
    print("       ② 波特率不对（手册默认 115200）")
    print("       ③ 这个 tty 不是机械臂")
PY
fi

# ------------------------------------------------------------------
hr; echo " 6/7  代码侧已知不一致（只打印，不改）"; hr
Y="$WS/src/yolov8_ros/script"
echo "  setup_kinematics 的参数（三处不一致 -> 同一目标点算出的关节角差 2~4°）："
grep -n "setup_kinematics(" "$Y/grab.py" "$Y/put.py" "$Y/keyboard_grasp.py" "$Y/arm_move.py" 2>/dev/null | sed 's/^/    /'
grep -n "setup_kinematics(" "$Y/z_kinematics.py" 2>/dev/null | grep -v "def " | sed 's/^/    /'
echo
echo "  夹爪 PWM（四处都不一样，必须现场定一个）："
grep -n '#005P' "$Y/grab.py" "$Y/put.py" "$Y/keyboard_grasp.py" 2>/dev/null | sed 's/^/    /'
echo
echo "  YOLO 订阅的话题 vs 相机发布的话题："
grep -n "Subscriber(" "$Y/multi_yolo.py" | sed 's/^/    /'
grep -n 'remap from="image"' "$WS/src/yolov8_ros/launch/usb_cam.launch" | sed 's/^/    /'
echo
echo "  抓取类别被写死成序列 car -> clock -> person（第 1/2/3 个货架必须按这个摆）："
grep -n 'label=' "$Y/grab.py" "$Y/put.py" | sed 's/^/    /'

# ------------------------------------------------------------------
hr; echo " 7/7  结论"; hr
echo "  本轮【模板识别 + dry_run】闸门："
echo "    ① /dev/arm 存在，机械臂串口后续能打开"
echo "    ② /dev/video0 和 /dev/video2 至少各有一个真实采集节点"
echo "    ③ 9 张水果模板齐全，find_object_2d 可解析"
echo "    ④ serial / rospy / cv2 / numpy 可导入"
echo
echo "  ai.sh 运行后再跑一次本脚本，应看到："
echo "    find_object_2d 在跑、ai_locate 在跑、/objects 和 /ai_locate/ids 有发布者"
echo
echo "  其他链路："
echo "    YOLO 权重只影响旧 multi_yolo/COCO 链，不影响水果模板 ai_grasp 链"
echo "    move_base 三层 enabled 只影响导航；没起 ROS 时显示跳过是正常"
echo
echo "  任一项不对都不要按 grasp.sh 第 7 页那个 1。"
echo
echo "============================================================"
echo " 体检结束    $(date '+%F %T')"
echo " 把整屏输出存成文件（放桌面 运行日志/ 里）："
echo "   ./7-grasp-check.sh 2>&1 | tee ~/grasp-check-\$(date +%Y%m%d-%H%M).log"
echo "============================================================"
