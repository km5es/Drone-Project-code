#!/bin/bash
#### Used within cron job to start MAVROS and get_metadata.py
##### author: Krishna Makhija
###### date: Sep 9th 2020
source ~/.bashrc
source /opt/ros/melodic/setup.bash

### wait for FC to boot up
sleep 8;

### start MAVROS

if ls /dev/ttySAC0 | grep -q 'ttySAC0'; then
    echo "GPIO UART detected..."
    mavlink-routerd -e 10.42.0.1:15550 -e 127.0.0.1:14550 /dev/ttySAC0:921600 &

elif ls /dev/ttyFC | grep -q 'ttyFC'; then
    echo "FTDI adapter detected..."
    mavlink-routerd -e 10.42.0.1:15550 -e 127.0.0.1:14550 /dev/ttyFC:921600 &

else
    echo "No MAVLink connection found..."
    exit
fi

roslaunch mavros apm.launch fcu_url:="udp://:14550@127.0.0.1:14551" &

sleep 8;

rosrun mavros mavsys rate --all 10 &

### start get_metadata.py
python ~/Drone-Project-code/Payload\ Computer/get_metadata.py &

