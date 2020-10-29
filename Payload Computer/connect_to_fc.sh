#!/bin/bash
#### Used within cron job to start MAVROS and get_metadata.py
##### author: Krishna Makhija
###### date: Sep 9th 2020
source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for FC to boot up
sleep 8;

### route MAVLink data 
mavlink-routerd -e 127.0.0.1:14550 -e 10.42.1.1:14550 -e 10.42.1.1:14551 /dev/ttyFC:921600 &

### start MAVROS
roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14550" &

sleep 8;

### start get_metadata.py
python ~/Drone-Project-code/Payload\ Computer/get_metadata.py &

