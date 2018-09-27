#!/bin/sh
xterm -e " cd ..; cd launch/; roslaunch turtlebot_wooduworld.launch " &
sleep 7
xterm -e " cd ..; cd launch/; roslaunch home_service.launch " &
sleep 7
xterm -e " cd ..; cd launch/; roslaunch amcl_woodu.launch " &
sleep 7
xterm -e " cd ..; cd launch/; roslaunch add_markers.launch " &
sleep 7
xterm -e " cd ..; cd launch/; roslaunch pick_objects.launch "
