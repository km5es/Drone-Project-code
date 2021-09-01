/* 
* C++ version of write_WPs.py
* This program writes WPs to the FCU upon bootup. When the drone has reached a WP
* and finished calibration, a ROS flag is set (trigger/waypoint) which causes this
* program to update the WP table on the FCU. The WP lookup table is in the ./mission/ 
* folder.
* 
* author: KM
* date: 1st Sep 2021
*/

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <fstream> 
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandHome.h>

// Define global variables
struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;   // get path to home directory
std::string filename = "/catkin_ws/src/Drone-Project-code/mission/mission.waypoints";

//? clear all waypoints on the FCU
void waypoint_clear_client(){
    ros::NodeHandle p;  
    ros::ServiceClient wp_clear_client = 
                p.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    mavros_msgs::WaypointClear wp_clear_srv;
    wp_clear_srv.request = {};
    if (wp_clear_client.call(wp_clear_srv)){
    ROS_INFO("Waypoint list was cleared");
    }
    else {
    ROS_ERROR("Waypoint list couldn't been cleared");
    }
}

/*
? Create a waypoint object to be sent to the FC.
?    Command: tells the drone what to do (navigate, land, RTL, etc)
?        22: takeoff
?        115: yaw
?        16: navigate to WP
?        19: loiter
?        20: RTL
?    param1: how to do it (delay at WP, yaw angle)
*/
mavros_msgs::Waypoint create_waypoint(int command, double param1, double latitude, 
                                                double longitude, double altitude){
    mavros_msgs::Waypoint wp_msg;
    wp_msg.frame = 0;  
    wp_msg.command = command;
    wp_msg.is_current = false;
    wp_msg.autocontinue = false;
    wp_msg.param1 = param1;
    wp_msg.param2 = 0;
    wp_msg.param3 = 0;
    wp_msg.param4 = 0;
    wp_msg.x_lat = latitude;
    wp_msg.y_long = longitude;
    wp_msg.z_alt = altitude;
    return wp_msg;
}

int main(int argc, char **argv){
    std::string &filenameptr = filename;
    filenameptr = homedir + filenameptr;
    ros::init(argc, argv, "write_WP");
    waypoint_clear_client();
}