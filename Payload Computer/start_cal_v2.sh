#!/bin/bash
kill -9 $(fuser /dev/ttyUSB0)
source ~/.bashrc
source /opt/ros/melodic/setup.bash

lsof -t -i tcp:8810 | xargs kill -9
cd ~/catkin_ws/src/Drone-Project-code/Payload\ Computer/

<< Python_version
sleep 35        # wait for connection to wifi hotspot
/usr/bin/python cal_sequence_tcp_server_v2.py &

sleep 1
/usr/bin/python gr_cal_tcp_loopback_client.py  

Python_version


#<<C++_version_testing

rosrun beam_mapping cal_seq &

#C++_version_testing