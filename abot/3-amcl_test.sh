###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /media/abot/999/abot/devel/setup.bash; roslaunch abot_bringup abot_bringup.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch cam_track usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; rosrun robot_slam pub_point.py; exec bash"' 
