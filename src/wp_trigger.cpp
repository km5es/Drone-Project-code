/*
* Re-writing some of the ROS nodes in C++ for better runtime efficiency.
* author: KM
*/ 

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/Waypoint.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <thread>

double vel_threshold = 0.35;
double pos_error_tol = 1.0;
double h;

double haversine(double lat1, double long1, double lat2, double long2){
    double r = 6.3781e6;
    double phi1 = lat1 * M_PI / 180;
    double phi2 = lat2 * M_PI / 180;
    double lam1 = long1 * M_PI / 180;
    double lam2 = long2 * M_PI / 180;
    double delta_phi = phi2 - phi1;
    double delta_lam = lam2 - lam1;
    double a = sin(delta_phi/2)*sin(delta_phi/2) + cos(lat1)*cos(lat2)*sin(delta_lam/2)*sin(delta_lam/2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double d = r * c;
    return d;
}

double haversine_3d(double lat1, double long1, double alt1, double lat2, double long2, double alt2){
    double alt_diff = alt2 - alt1;
    double d_2d = haversine(lat1, long1, lat2, long2);
    double d_3d = sqrt(pow(d_2d, 2) + pow(alt_diff, 2));
    return d_3d;
}

void get_velocity(const nav_msgs::Odometry::ConstPtr& msg){
    double x_vel = msg->twist.twist.linear.x;
    double y_vel = msg->twist.twist.linear.y;
    double z_vel = msg->twist.twist.linear.z;
    double v = sqrt(pow(x_vel, 2) + pow(y_vel, 2) + pow(z_vel, 2));
    ROS_INFO("v = %f", v);
}

std::vector<mavros_msgs::Waypoint> wp_list;
void get_waypoints(const mavros_msgs::WaypointList& msg){
    //std::vector<mavros_msgs::Waypoint> wp_list = msg.waypoints;
    wp_list = msg.waypoints;
    ROS_INFO("Retrieved WP list");
    ROS_INFO("The current target WP coords are %f, %f, and %f", wp_list[1].x_lat, wp_list[1].y_long, wp_list[1].z_alt);
}

void get_haversine(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (msg->status.status == 0){
        h = haversine(msg->latitude, msg->longitude, wp_list[1].x_lat, wp_list[1].y_long);
        std::cout << "Distance to next WP is: " << h << " m." << std::endl;
    }
    else if (msg->status.status == -1){
        ROS_ERROR("GPS signal not available.");
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "wp_trigger");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/mavros/mission/waypoints", 1000, get_waypoints);
    ros::Subscriber sub2 = n.subscribe("/mavros/global_position/global", 1000, get_haversine);
    //ros::Subscriber sub3 = n.subscribe("/mavros/local_position/pose", 1000, get_distance);
    ros::Subscriber sub4 = n.subscribe("/mavros/local_position/odom", 1000, get_velocity);
    ros::spin();
    return 0;
}
