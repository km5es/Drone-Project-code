## ROS node for detecting waypoint arrival
The ROS node, [drone_project.py](drone_project.py) needs to be built inside a catkin workspace to function properly.
```
cd ~/catkin_ws/
catkin_make
```
To run:
```
rosrun flying_spider drone_project.py
```
Its purpose is to interpret MAVLink data to identify when the drone reaches a waypoint by reading WPreached.msg and setting a flag in the base station [code](https://github.com/km5es/Drone-Project/tree/master/Base%20Station/open-loop-accept-tcp.py) to begin the sequence.

>**NOTE**: Tested on ROS Melodic.