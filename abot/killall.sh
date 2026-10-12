#!/bin/bash
# ============================================================
#  ABOT 一键清理 ROS 进程
#
#  修订历史：
#   · 2026-10-10 整脚本重写。旧版只有 killall rosmaster / roscore 两行，
#     它【不杀 roslaunch，也不杀任何子节点】。结果 rplidarNode、abot_driver
#     等一直活着占着串口，下一次启动时串口被占，雷达起不来、amcl 报
#     "No laser scan received"。这就是 C3 的真正触发机制。
#
#   · 2026-10-12 补上"工作区节点"这一层 —— 10-10 那版只算半个修复。
#     它第二层是 pkill -f "/opt/ros/noetic/lib"，只覆盖 ROS【系统自带】的节点。
#     而本工作区自己编译出来的节点根本不在那个路径下：
#         /media/abot/999/abot/devel/lib/abot_bringup/abot_driver
#     所以 abot_driver 每次都活下来 —— 2026-10-12 14:45 上机实测抓到现行：
#         [!!] abot_driver 还有 1 个残留进程：16155 .../devel/lib/abot_bringup/abot_driver
#     同样漏网的还有【源码方式跑的 python 节点】（.../src/robot_slam/scripts/*.py）：
#     它们的命令行里既没有 "rosrun"，也没有 "/opt/ros/noetic/lib"。
#     本版加第 3 层专门覆盖这两种形态，并加串口兜底。
#
#  原则：宁可多杀几个自己人，也不要漏掉一个占着串口的驱动 ——
#        清理脚本漏杀的代价比误杀大得多。
#
#  用法：
#      cd /media/abot/999/abot
#      ./killall.sh
# ============================================================

# 工作区根目录 = 本脚本所在目录。用推导而不是写死，
# 这样 U 盘挂载点变了（/media/abot/<卷标>/...）也不用改脚本。
WS="$(cd "$(dirname "$0")" && pwd)"

# fuser 一般装在 /usr/sbin，普通用户的 PATH 里往往【没有】sbin —— 所以要显式找。
# （2026-10-12 上机实测：直接敲 `fuser` 报「未找到命令」，但 /usr/sbin/fuser 是有的。）
FUSER=""
for c in fuser /usr/sbin/fuser /sbin/fuser /bin/fuser /usr/bin/fuser; do
  if command -v "$c" >/dev/null 2>&1; then FUSER="$c"; break; fi
done

echo "正在清理 ROS 进程..."
echo "（工作区: $WS）"
echo

# 1) roslaunch / rosrun 本身，以及 gnome-terminal 里的 bash -c 包装进程
pkill -9 -f "roslaunch" 2>/dev/null
pkill -9 -f "rosrun"    2>/dev/null

# 2) ROS 系统自带的 C++ 节点（rplidarNode、rviz、map_server、move_base ...）
pkill -9 -f "/opt/ros/noetic/lib" 2>/dev/null

# 3) 【2026-10-12 新增】本工作区自己的节点，两种形态都要覆盖
pkill -9 -f "$WS/devel/lib/"   2>/dev/null   # 编译出来的 C++ 节点（abot_driver 在这）
pkill -9 -f "$WS/src/"         2>/dev/null   # 源码跑的 python 节点（pub_point.py 在这）
# 标签页里那个还停在 sleep 的包装进程 —— 杀掉它，免得清理完才慢悠悠把节点拉起来
pkill -9 -f "$WS/devel/setup.bash" 2>/dev/null

# 4) 会话与仿真
killall -9 rosmaster 2>/dev/null
killall -9 roscore   2>/dev/null
killall -9 rosout    2>/dev/null
killall -9 gzserver  2>/dev/null
killall -9 gzclient  2>/dev/null

# 5) 兜底：谁还占着串口就杀谁（C3 的本质就是"串口被占"，
#    不管对方叫什么名字、从哪来的，占着串口就是错的）
if [ -n "$FUSER" ]; then
  for dev in /dev/abot_driver /dev/rplidar; do
    # 只对【真实存在】的设备节点动刀：悬空软链接会让 fuser 报错
    [ -e "$dev" ] && "$FUSER" -k -9 "$dev" 2>/dev/null
  done
else
  echo "  [--]   找不到 fuser，跳过串口兜底（不影响上面的清理）"
fi

sleep 1
echo

# 6) 复查 —— 下面每一项都应该是 [OK]
#    注意：这里必须用【启动后再杀】的方式验证。空的 pgrep 有两种可能：
#    一种是杀干净了，另一种是根本没东西起来（2026-10-12 就栽在这上面）。
echo "复查（应该全部 [OK]）:"
for p in rplidar abot_driver roslaunch rosrun move_base map_server \
         pub_point multi_yolo abot_bringup; do
  n=$(pgrep -f "$p" 2>/dev/null | wc -l)
  if [ "$n" -eq 0 ]; then
    echo "  [OK]   $p 已清理"
  else
    echo "  [!!]   $p 还有 $n 个残留进程:"
    pgrep -a -f "$p" 2>/dev/null | sed 's/^/           /'
  fi
done

echo
echo "串口占用情况（应该是空的）:"
if [ -n "$FUSER" ]; then
  held=$("$FUSER" /dev/abot_driver /dev/rplidar 2>/dev/null)
  if [ -z "$held" ]; then
    echo "  [OK]   两个串口都没人占"
  else
    echo "  [!!]   串口仍被占用，PID:$held"
  fi
else
  echo "  [--]   找不到 fuser，无法检查"
fi

echo
echo "串口/设备节点:"
# ★ 2026-10-12 教训：软链接存在 ≠ 设备存在。悬空软链接会骗过一切只看 ls 的检查。
echo "  /dev/ttyUSB*：$(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ' || true)"
for dev in /dev/abot_driver /dev/rplidar; do
  if [ ! -e "$dev" ] && [ ! -L "$dev" ]; then
    echo "  [!!]   $dev 不存在 —— 适配器没枚举出来，检查 USB 线和供电"
  elif [ ! -e "$dev" ]; then
    echo "  [!!]   $dev -> $(readlink "$dev") 是【悬空软链接】！目标 /dev/ttyUSB* 不存在"
    echo "         设备已经掉了 —— 重新插拔 USB 或重启，然后确认 /dev/ttyUSB* 出现"
  else
    echo "  [OK]   $dev -> $(readlink -f "$dev")（真实存在）"
  fi
done

echo
echo "清理结束。若上面还有 [!!]，把截图发我。"
