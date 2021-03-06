#!/bin/bash
#### A convenience script to run the entire SITL autonomy pipeline.
###### Author: KM
######## Date: July 16th 2020

### clear comms
lsof -t -i udp:14540 | xargs kill -9
kill -9 $(fuser /dev/ttyUSB0)
lsof -t -i tcp:8800 | xargs kill -9

### run MAVROS
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" &

### run PX4 simulator in headless mode
gnome-terminal -e "bash -c 'cd ~/src/Firmware/ && HEADLESS=1 make px4_sitl gazebo_solo;'"

### run MAVROS
#gnome-terminal -e "bash -c 'roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"'"

sleep 2.5;

### run ROS node
gnome-terminal -e "bash -c 'rosrun beam_mapping drone_project.py;'"

### run GRC TCP server
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/tcp_toggle.py;'"

sleep 6;

### run GRC TCP client
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/base-station-receiver.py;'"

### run QGroundControl
~/Downloads/QGroundControl.AppImage