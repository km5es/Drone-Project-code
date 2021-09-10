#!/bin/bash
#### ROS nodes for handling mission
##### author: Krishna Makhija
###### date: Mar 5th 2021

source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for MAVROS and FCU to boot up
sleep 30

<<python_version
cd ~/catkin_ws/src/Drone-Project-code/Payload\ Computer/
### ROS node for updating WP table
/usr/bin/python write_WPs.py &
sleep 1
### ROS node for detecting if WP is reached
/usr/bin/python wp_trigger.py &
sleep 1
### ROS node for saving metadata
/usr/bin/python get_metadata.py &
python_version

#<<C++_version
### ROS node for updating WP table
rosrun beam_mapping write_WPs &
sleep 1
### ROS node for detecting if WP is reached
rosrun beam_mapping wp_trigger &
sleep 1
### ROS node for saving metadata
rosrun beam_mapping get_metadata &
#C++_version