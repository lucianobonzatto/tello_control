#!/bin/bash

chmod +x multi_tello_connect.sh
chmod +x echo_view.sh
chmod +x manager.sh
chmod +x pid_ar_tracker.sh

gnome-terminal --window -- ./multi_tello_connect.sh
sleep 7
gnome-terminal --window -- ./echo_view.sh
gnome-terminal --window -- ./manager.sh
gnome-terminal --window -- ./pid_ar_tracker.sh