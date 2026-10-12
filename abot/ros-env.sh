#!/bin/bash
# ============================================================
#  ROS 网络变量覆盖 —— 修 C5
#
#  背景：这台机器的环境里预置了
#      ROS_HOSTNAME=172.20.10.2
#      ROS_MASTER_URI=http://172.20.10.2:11311
#  那是【旧手机热点】的网段。机器真实 IP 一变（现在是 10.128.202.88），
#  ROS 就对外宣告一个本机不存在的地址 → roscore 连自己都连不上、启动即退出：
#      RLException: Unable to contact my own server at [http://172.20.10.2:36113]
#  症状很迷惑人：rosnode list 不是报错，而是【无限卡住】。
#
#  证据（2026-10-12 实测）：标签页里的 roscore 报出的正是环境变量里的
#  172.20.10.2，而不是本机真实 IP ⇒ 说明 gnome-terminal 开的标签页会继承
#  调用者的环境 ⇒ 在启动脚本里 export 一次就够了。
#
#  为什么用 localhost 而不是本机 IP：全部节点都跑在本机；而且这里是手机
#  热点/校园网混用，IP 会变，写死 localhost 最稳。
#  将来若要从另一台机器看 RViz，再改成动态 IP 方案。
#
#  用法：在启动脚本里 `source .../ros-env.sh`（或直接 export 这两行）。
# ============================================================

export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
