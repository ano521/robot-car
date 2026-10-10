gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/abot/devel/setup.bash; roslaunch abot_bringup abot_bringup.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch yolov8_ros multi_yolo.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; rosrun yolov8_ros grab.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; rosrun yolov8_ros put.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch robot_slam grab.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \

