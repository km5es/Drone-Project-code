#!/bin/bash
#kill -9 $(fuser /dev/ttyUSB0)
source ~/.bashrc
source /opt/ros/melodic/setup.bash

lsof -t -i tcp:8810 | xargs kill -9
cd ~/Drone-Project-code/Payload\ Computer/
python cal_sequence_tcp_server_v2.py &
python gr_cal_tcp_loopback_client.py &
