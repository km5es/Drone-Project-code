#!/bin/bash
#cd ~/src/Firmware/ && HEADLESS=1 make px4_sitl gazebo_solo &
#sleep 8
#tilix -a session-add-down -x "bash -c 'source /opt/ros/melodic/setup.bash; source /home/kmakhija/catkin_ws/devel/setup.bash; roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"; exec $SHELL'"

### run MAVROS
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" &

sleep 5

### run ROS node
tilix -a session-add-right -x "bash -c 'source /opt/ros/melodic/setup.bash;\
source ~/catkin_ws/devel/setup.bash;\
source ~/.bashrc;\
rosrun flying_spider drone_project.py; exec $SHELL'"

### run PX4 simulator in headless mode
tilix -a session-add-down -x "bash -c 'export PX4_HOME_LAT=37.994125;\
export PX4_HOME_LON=-78.397535;\
export PX4_HOME_ALT=28.5;\
cd ~/src/Firmware/ && HEADLESS=1 make px4_sitl gazebo_solo; exec $SHELL'"

### 
tilix -a session-add-left -x "bash -c 'python ~/Drone-Project/Base\ Station/tcp_toggle.py; exec $SHELL'"