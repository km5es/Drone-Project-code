#!/bin/bash
#### A convenience script to run the entire autonomy pipeline during the experiment.
###### Author: KM
######## Date: Mar 5th 2021

### clear any previously running scripts
lsof -t -i tcp:8800 | xargs kill -9

roscore &

#### begin ROS script
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/get_sdr_temp.py;'"


#### run GRC TCP server
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/tcp_toggle.py;'"

sleep 6;

#### run GRC TCP client

gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/base-station-receiver.py;'"

#### run QGroundControl
~/Downloads/QGroundControl.AppImage