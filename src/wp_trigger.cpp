/*
* C++ version of wp_trigger.py
* This runs in the background to check if drone is within specc'ed distance of WP and stable 
* enough prior to calibration. It begins the entire sequence by setting the ("trigger/sequence")
* flag.

* author: KM
* date: 28th Aug 2021
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
#include <mutex>
#include <unistd.h>

double vel_threshold    = 0.35;     // linear velocity threshold for starting cal (m/s)
double pos_error_tol    = 1.0;      // acceptable 3D radius around setpoint to start cal (m)
double wp_wait_timeout  = 10.0;     // sleep code for this much time after flag is set (s)
double h, v;                        // global 2D haversince distance and linear velocity (m, m/s)
std::vector<mavros_msgs::Waypoint> wp_list;
std::mutex h_mtx;                   // haversine calculation event

// Calculate 2D haversine distance between two points.
double haversine(double lat1, double long1, double lat2, double long2){
    double r = 6.3781e6;            // radius of earth (m)
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

// Calculate 3D haversine distance between two points.
double haversine_3d(double lat1, double long1, double alt1, double lat2, double long2, double alt2){
    double alt_diff = alt2 - alt1;
    double d_2d = haversine(lat1, long1, lat2, long2);
    double d_3d = sqrt(pow(d_2d, 2) + pow(alt_diff, 2));
    return d_3d;
}

// Retrieve linear velocities in x, y, and z, and calculate the magnitude of all three directions
void get_velocity(const nav_msgs::Odometry::ConstPtr& msg){
    double x_vel = msg->twist.twist.linear.x;
    double y_vel = msg->twist.twist.linear.y;
    double z_vel = msg->twist.twist.linear.z;
    double v = sqrt(pow(x_vel, 2) + pow(y_vel, 2) + pow(z_vel, 2));
    //ROS_INFO("v = %f", v);
}

// Retrieve current waypoints on the FCU
//! make sure the indices are correct here
void get_waypoints(const mavros_msgs::WaypointList& msg){
    wp_list = msg.waypoints;
    ROS_INFO("Retrieved WP list");
    ROS_INFO("The current target WP coords are %f, %f, and %f", wp_list[1].x_lat, wp_list[1].y_long, wp_list[1].z_alt);
}

// Calculate in real-time what the 2D distance is to the current WP
void get_haversine(const sensor_msgs::NavSatFix::ConstPtr& msg){
    if (msg->status.status == 0){
        h = haversine(msg->latitude, msg->longitude, wp_list[1].x_lat, wp_list[1].y_long);
        //ROS_INFO("Distance to next WP is %f m", h);
        h_mtx.lock();   // lock this thread so that 3D distance can be computed
    }
    else if (msg->status.status == -1){
        ROS_ERROR("GPS signal not available.");
    }
}

// Calculate in real-time what teh 3D distance is to the current WP
void get_distance(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (!h_mtx.try_lock()){     // ensure that 2D distance is first calculated
        double alt_diff = wp_list[1].z_alt - msg->pose.position.z;
        double distance = sqrt(pow(distance, 2) + pow(h, 2));
        // check to see if drone stability conditions are met
        if (distance < pos_error_tol && v <= vel_threshold){
            ROS_INFO(">>>>WP reached<<< ||| Drone is stable and (almost) not moving.");
            // this begins the whole sequence. will trigger/metadata next.
            ros::param::set("trigger/sequence", true);
            sleep(wp_wait_timeout); // ! can I have the sleep be interrupted by get_waypoints()?
        }
        h_mtx.unlock();         // release lock so that 2D distance can be calculated
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "wp_trigger");
    ros::NodeHandle n;
    n.setParam("trigger/sequence", false);
    ros::Subscriber sub1 = n.subscribe("/mavros/local_position/odom", 1000, get_velocity);
    ros::Subscriber sub2 = n.subscribe("/mavros/mission/waypoints", 1000, get_waypoints);
    ros::Subscriber sub3 = n.subscribe("/mavros/global_position/global", 1000, get_haversine);
    ros::Subscriber sub4 = n.subscribe("/mavros/local_position/pose", 1000, get_distance);
    ros::spin();
    return 0;
}
