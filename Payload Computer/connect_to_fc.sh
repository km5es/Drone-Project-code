#!/bin/bash
#### Used within cron job to start MAVROS and get_metadata.py
##### author: Krishna Makhija
###### date: Sep 9th 2020
source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for FC to boot up
sleep 8;

### start MAVROS
roslaunch mavros apm.launch fcu_url:="/dev/ttyFC:921600" &

sleep 8;

### start get_metadata.py
python ~/Drone-Project-code/Payload\ Computer/get_metadata.py &

