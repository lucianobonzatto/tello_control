#!/bin/bash

chmod +x ar_track.sh
chmod +x pid_control.sh
chmod +x tello_control.sh


gnome-terminal --window -- ./ar_track.sh
sleep 10

gnome-terminal --window -- ./pid_control.sh
sleep 5

gnome-terminal --window -- ./tello_control.sh