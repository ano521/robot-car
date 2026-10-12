###gmapping with abot###
# --- ROS 网络变量覆盖（修 C5，2026-10-12）---
# 环境里预置的 ROS_HOSTNAME=172.20.10.2 / ROS_MASTER_URI=http://172.20.10.2:11311
# 是【旧手机热点】网段；机器真实 IP 一变，roscore 就连不上自己、启动即退出，
# 症状是 rosnode list 无限卡住（不报错，就是干等）。
# 下面两行显式覆盖掉。已实测 gnome-terminal 的标签页会继承调用者的环境
# （证据：标签页里的 roscore 报的正是环境变量里的 172.20.10.2），
# 所以放在 gnome-terminal 之前就够了。细节见 abot/ros-env.sh
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /media/abot/999/abot/devel/setup.bash; roslaunch abot_bringup abot_bringup.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam grab.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \

