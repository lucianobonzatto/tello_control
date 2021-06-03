#!/bin/sh
xterm  -e  "source ~/catkin_ws/devel/setup.bash;roslaunch tello_driver tello_node.launch" &
sleep 1
xterm  -e  "source ~/catkin_ws/devel/setup.bash;roslaunch tello_control joy_control.launch" &
