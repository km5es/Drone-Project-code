#!/bin/bash
#### A convenience script to run the entire autonomy pipeline during the experiment.
###### Author: KM
######## Date: Sep 13th 2020

### clear any previously running scripts
lsof -t -i tcp:8800 | xargs kill -9

### intitate MAVLink with drone

if [[ $# -lt 1 ]]; then
	echo "Not enough arguments. Please specify whether to connect via wifi or telemetry."
    echo "Example: "
    echo "./begin_mission.sh wifi"
	exit
fi

if [ $1 = "wifi" ]; then
	echo "Connecting to drone via $1"
#    roslaunch mavros apm.launch fcu_url:="udp://:15550@127.0.0.1:15551" &
    gnome-terminal -e "bash -c 'roslaunch mavros apm.launch fcu_url:=udp://:15550@10.42.0.1:15551 gcs_url:=udp://@127.0.0.1:17550'" 
    sleep 6
    rosrun mavros mavsys rate --all 10

elif [ $1 = "telemetry" ]; then
	echo "Connecting to drone via $1"
    gnome-terminal -e "bash -c 'roslaunch mavros apm.launch fcu_url:=/dev/ttyMAVROS:57600 gcs_url:=udp://@127.0.0.1:17550'"
    sleep 6
    rosrun mavros mavsys rate --all 10

else
    echo "Please specify if connection is telemetry or wifi."
    exit
fi


#### begin ROS script
gnome-terminal -e "bash -c 'rosrun beam_mapping ros-trigger.py;'"


#### run GRC TCP server
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/tcp_toggle.py;'"

#sleep 6;

#### run GRC TCP client
gnome-terminal -e "bash -c 'python ~/catkin_ws/src/Drone-Project-code/Base\ Station/base-station-receiver.py -n $1;'"

#### run QGroundControl
~/Downloads/QGroundControl.AppImage