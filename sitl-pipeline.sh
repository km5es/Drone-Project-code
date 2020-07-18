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

sleep 2.5

### run ROS node
gnome-terminal -e "bash -c 'source /opt/ros/melodic/setup.bash;\
source ~/catkin_ws/devel/setup.bash;\
source ~/.bashrc;\
rosrun beam_mapping drone_project.py; exec $SHELL'"

### run PX4 simulator in headless mode
gnome-terminal -e "bash -c 'export PX4_HOME_LAT=37.994125;\
export PX4_HOME_LON=-78.397535;\
export PX4_HOME_ALT=28.5;\
cd ~/src/Firmware/ && HEADLESS=1 make px4_sitl gazebo_solo; exec $SHELL'"

### run GRC TCP server
gnome-terminal -e "bash -c 'python ~/Drone-Project/Base\ Station/tcp_toggle.py; exec $SHELL'"

sleep 6;

### run GRC TCP client
gnome-terminal -e "bash -c 'python ~/Drone-Project/Base\ Station/open-loop-accept-tcp.py; exec $SHELL'";

### run QGroundControl
gnome-terminal -e "bash -c '~/Downloads/QGroundControl.AppImage; exec $SHELL'"