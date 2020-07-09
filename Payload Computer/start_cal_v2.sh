#!/bin/bash
sleep 5
cd ~/Drone-Project/Payload\ Computer/
python cal_sequence_tcp_server.py &
python gr_cal_tcp_loopback_client.py &
