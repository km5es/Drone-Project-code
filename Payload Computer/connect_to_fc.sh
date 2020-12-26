#!/bin/bash
#### Used within cron job to start MAVROS and get_metadata.py
##### author: Krishna Makhija
###### date: Sep 9th 2020
bash -c 'source ~/.bashrc'
bash -c 'source /opt/ros/$(rosversion -d)/setup.bash'

### wait for FC to boot up
sleep 8;

### start MAVROS
roslaunch mavros px4.launch fcu_url:="/dev/ttyFC:921600" &

sleep 8;

### start get_metadata.py
python ~/Drone-Project-code/Payload\ Computer/get_metadata.py &

