#!/bin/sh
xterm -e " cd ../../..; cd turtlebot_simulator/turtlebot_gazebo/launch/; roslaunch turtlebot_world.launch " &
sleep 1
xterm -e " roslaunch gmapping_demo.launch " &
# sleep 1
# xterm -e " cd ../../..; cd turtlebot_interactions/turtlebot_rivz_launchers/launch/; roslaunch view_navigation.launch " &
# sleep 1
# xterm -e " cd ..; cd turtlebot/turtlebot_teleop/launch/; roslaunch keyboard_teleop.launch "
