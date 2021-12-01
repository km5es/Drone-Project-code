/*
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg){

    ROS_INFO("\nlinear acceleration\
                \nx: [%f]\ny:[%f]\nz:[%f]\nang_x: [%f]", msg->linear_acceleration.x,
                msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x);

    // std::cout << msg->linear_acceleration.z << "\n";
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    //ROS_INFO("\nangular acceleration\nx: [%f]", msg->twist.twist.angular.x);
    ROS_INFO("orientation:\nx = [%f]\n y = [%f]\n z = [%f]\n w = [%f]\n", 
                msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "imu_data");
    ros::NodeHandle n;
    //ros::Subscriber sub1 = n.subscribe("/mavros/imu/data", 1000, chatterCallback);
    ros::Subscriber sub2 = n.subscribe("/mavros/local_position/odom", 1000, odom_callback);
    ros::spin();
    return 0;
}
*/

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <fstream> 
#include <time.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>
#include <exception>
#include <vector>
#include <iterator>
#include <sstream>
#include <iomanip>
#include <stdexcept>        // std::out_of_range
#include <stdlib.h>         // EXIT_SUCCESS         
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandHome.h>

// Define global variables
mavros_msgs::Waypoint wp;           // WPs object


//? Create a waypoint object to be sent to the FC.
//?    Command: tells the drone what to do (navigate, land, RTL, etc)
//?        22: takeoff
//?        115: yaw
//?        16: navigate to WP
//?        19: loiter
//?        20: RTL
//?    param1: how to do it (delay at WP, yaw angle)
mavros_msgs::Waypoint create_waypoint(int command, double param1, double latitude, 
                                                double longitude, double altitude){
    mavros_msgs::Waypoint wp_msg;
    wp_msg.frame = 3;  
    wp_msg.command = command;
    wp_msg.is_current = false;
    wp_msg.autocontinue = true;
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
    ros::init(argc, argv, "write_WP");
    ros::NodeHandle nh;
    ros::ServiceClient wp_client = 
                nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv;          // List of Waypoints
    mavros_msgs::Waypoint wp;                       // WPs object
    int n = 1;                                      // wp counters
    //? create initial WP on bootup
    wp = create_waypoint(22, 0, 37.5, -78.5, 10);
    wp_push_srv.request.waypoints.push_back(wp);
    wp = create_waypoint(115, 0, 37.5, -78.5, 10);
    wp_push_srv.request.waypoints.push_back(wp);
    wp = create_waypoint(16, 0, 37.5, -78.5, 10);
    wp_push_srv.request.waypoints.push_back(wp);
    wp_client.call(wp_push_srv);
    wp_push_srv.request.waypoints.clear();                    // this pushes WP list to FCU
    wp_client.call(wp_push_srv);
    std::cout << wp_push_srv.request.waypoints[0].x_lat << std::endl;
    return 0;
}