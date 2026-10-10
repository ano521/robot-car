#!/bin/bash
# ============================================================
#  ABOT 一键清理 ROS 进程
#
#  为什么重写：旧版只有 killall rosmaster / roscore 两行，
#  它【不杀 roslaunch，也不杀任何子节点】。结果 rplidarNode、
#  abot_driver 这些一直活着占着 /dev/ttyUSB0，下一次启动时
#  串口被占，雷达起不来，amcl 报 "No laser scan received"。
#  这就是上机记录里 C3 的根因。
#
#  用法：
#      cd /media/abot/999/abot
#      ./killall.sh
# ============================================================

echo "正在清理 ROS 进程..."

# 1) roslaunch / rosrun 本身，以及 gnome-terminal 里的 bash -c 包装进程
pkill -9 -f "roslaunch" 2>/dev/null
pkill -9 -f "rosrun"    2>/dev/null

# 2) ROS 自带的 C++ 节点（rplidarNode、底盘驱动、rviz、map_server、move_base ...）
pkill -9 -f "/opt/ros/noetic/lib" 2>/dev/null

# 3) 会话与仿真
killall -9 rosmaster 2>/dev/null
killall -9 roscore   2>/dev/null
killall -9 rosout    2>/dev/null
killall -9 gzserver  2>/dev/null
killall -9 gzclient  2>/dev/null

sleep 1
echo

# 4) 复查 —— 下面每一项都应该是 [OK]
echo "复查（应该全部 [OK]）:"
for p in rplidar abot_driver roslaunch rosrun move_base map_server; do
  n=$(pgrep -f "$p" 2>/dev/null | wc -l)
  if [ "$n" -eq 0 ]; then
    echo "  [OK]   $p 已清理"
  else
    echo "  [!!]   $p 还有 $n 个残留进程:"
    pgrep -a -f "$p" 2>/dev/null | sed 's/^/           /'
  fi
done

echo
echo "串口/设备节点:"
ls -l /dev/abot_driver /dev/rplidar 2>/dev/null | sed 's/^/  /' \
  || echo "  （串口设备节点不存在——检查硬件连接）"

echo
echo "清理结束。若上面还有 [!!]，把截图发我。"
