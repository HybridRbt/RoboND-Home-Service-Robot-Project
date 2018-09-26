#!/bin/sh
xterm -e " cd ..; cd launch/; roslaunch add_markers.launch " &
sleep 5
xterm -e " cd ..; cd launch/; roslaunch pick_objects.launch "
sleep 5
xterm -e " rostopic info arrived_flag "
