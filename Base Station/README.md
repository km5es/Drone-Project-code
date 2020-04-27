### Base station codes
The tcp_toggle.py file has a multi_usrp block which writes data via tcp loopback to the open-loop-accept-tcp.py file which has a ROS node built-in. 
ROS node looks for drone position, begins saving data from tcp, and triggers the payload computer. 
