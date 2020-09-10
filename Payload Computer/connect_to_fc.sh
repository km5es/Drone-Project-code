#!/bin/bash
#### Used within cron job to start MAVROS and get_metadata.py
##### author: Krishna Makhija
###### date: Sep 9th 2020

### wait for FC to boot up
sleep 8;

### start MAVROS
roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:57600" &

sleep 5;

### start get_metadata.py
gnome-terminal -e "bash -c 'python get_metadata.py;'"

