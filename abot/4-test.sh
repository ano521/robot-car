###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source /media/abot/999/abot/devel/setup.bash; roslaunch abot_bringup abot_bringup.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam grab.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source /media/abot/999/abot/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \

