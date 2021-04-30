#!/bin/bash
#### ROS nodes for handling mission
##### author: Krishna Makhija
###### date: Mar 5th 2021

source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for MAVROS and FCU to boot up
sleep 16

### ROS node for updating WP table
python ~/Drone-Project-code/Payload\ Computer/write_WPs.py &
### ROS node for detecting if WP is reached
python ~/Drone-Project-code/Payload\ Computer/wp_trigger.py &
### ROS node for saving metadata
python ~/catkin_ws/src/Drone-Project-code/Payload\ Computer/get_metadata.py &