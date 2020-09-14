#!/bin/bash
#### A convenience script to run the entire autonomy pipeline during the experiment.
###### Author: KM
######## Date: Sep 13th 2020

### clear any previously running scripts
lsof -t -i tcp:8800 | xargs kill -9

### intitate MAVLink with drone
roslaunch mavros px4.launch fcu_url:="/dev/ttyMAVROS:57600" &

### begin ROS script
gnome-terminal -e "bash -c 'rosrun beam_mapping drone_project.py;'"


### run GRC TCP server
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/tcp_toggle.py;'"

sleep 6;

### run GRC TCP client
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/base-station-receiver.py;'"

### run QGroundControl
~/Downloads/QGroundControl.AppImage