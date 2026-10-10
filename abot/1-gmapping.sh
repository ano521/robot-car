###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/abot/devel/setup.bash; roslaunch abot_bringup abot_bringup.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch robot_slam gmapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot/devel/setup.bash; roslaunch robot_slam view_mapping.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash"' \ 

