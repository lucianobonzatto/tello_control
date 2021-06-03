#!/bin/sh
xterm  -e  "source ~/catkin_ws/devel/setup.bash;roslaunch tello_control joy_control.launch" &
sleep 2
xterm  -e  "source ~/catkin_ws/devel/setup.bash;roslaunch tello_driver tello_node.launch" &
