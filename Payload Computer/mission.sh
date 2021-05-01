#!/bin/bash
#### ROS nodes for handling mission
##### author: Krishna Makhija
###### date: Mar 5th 2021

source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for MAVROS and FCU to boot up
sleep 30

cd ~/catkin_ws/src/Drone-Project-code/Payload\ Computer/
### ROS node for updating WP table
python write_WPs.py &
sleep 1
### ROS node for detecting if WP is reached
python wp_trigger.py &
sleep 1
### ROS node for saving metadata
python get_metadata.py &