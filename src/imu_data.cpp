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