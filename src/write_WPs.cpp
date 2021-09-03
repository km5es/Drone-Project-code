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
#include <exception>
#include <vector>
#include <iterator>
#include <sstream>
#include <iomanip>
#include <stdexcept>      // std::out_of_range
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandHome.h>

// Define global variables
bool seq_flag, way_flag;
struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;   // get path to home directory
std::string filename = "/catkin_ws/src/Drone-Project-code/mission/mission.waypoints";
mavros_msgs::Waypoint wp;           // WPs object

//? split strings according to specified delimiter
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

//? toggle modes before departing each WP. The WP table needs to be cleared 
//? before updating with a new WP and switching "back" to Auto.
void change_mode(){
    ros::NodeHandle l;
    ros::ServiceClient change_mode_client = 
                            l.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode brake_set_mode;
    mavros_msgs::SetMode auto_set_mode;
    brake_set_mode.request.custom_mode = "BRAKE";
    auto_set_mode.request.custom_mode = "AUTO";
    change_mode_client.call(brake_set_mode);
    change_mode_client.call(auto_set_mode);
    ROS_INFO("Toggling modes on copter.");
}

//? clear all waypoints on the FCU
void waypoint_clear_client(){
    ros::NodeHandle m;  
    ros::ServiceClient wp_clear_client = 
                m.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
    mavros_msgs::WaypointClear wp_clear_srv;
    wp_clear_srv.request = {};
    if (wp_clear_client.call(wp_clear_srv)){
    ROS_INFO("Waypoint list was cleared");
    }
    else {
    ROS_ERROR("Waypoint list couldn't been cleared");
    }
}

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
    // ? arrange data from mission.waypoints into separate vectors.
    //? this seems goofy. maybe there is a better way to do this?
    std::string &filenameptr = filename;
    filenameptr = homedir + filenameptr;
    std::ifstream mission_file(filename);                       // load mission files
    std::string line;
    std::vector<double> command, param1, latitude, longitude, altitude;
    if (mission_file.is_open()){
        ROS_INFO("Mission file retrieved from: %s", filename.c_str());
    }
    else {
        ROS_ERROR("Failed to open mission file.");
    }
    std::getline(mission_file, line);                           // skip the first line
    while (std::getline(mission_file, line)){                   // while each line is read
        std::vector<std::string> data_per_row;
        split(line, '\t', data_per_row);                        // each line is separate entry
        for (auto v: data_per_row)
            //std::cout << v << "\t" ;
            //std::cout << std::endl;;
            command.push_back(std::stod(data_per_row[3]));
            param1.push_back(std::stod(data_per_row[4]));       // creating separate vectors 
            latitude.push_back(std::stod(data_per_row[8]));     //   for separate args to 
            longitude.push_back(std::stod(data_per_row[9]));    //      create_waypoint()
            altitude.push_back(std::stod(data_per_row[10]));
    }
    ros::init(argc, argv, "write_WP");
    ros::NodeHandle nh;
    ros::ServiceClient wp_client = 
                nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv;          // List of Waypoints
    mavros_msgs::Waypoint wp;                       // WPs object
    waypoint_clear_client();
    int n = 1;                                      // wp counters
    //? create initial WP on bootup
    wp = create_waypoint(22, 0, latitude[0], longitude[0], altitude[0]);
    wp_push_srv.request.waypoints.push_back(wp);
    wp = create_waypoint(115, param1[n+1], latitude[n], longitude[n], altitude[n]);
    wp_push_srv.request.waypoints.push_back(wp);
    wp = create_waypoint(16, 0, latitude[n], longitude[n], altitude[n]);
    wp_push_srv.request.waypoints.push_back(wp);
    wp_client.call(wp_push_srv);                    // this pushes WP list to FCU
    while (ros::ok()){
        usleep(5e4);
        if (nh.param("trigger/sequence", seq_flag) == true){
            waypoint_clear_client();
            nh.setParam("trigger/metadata", true);                  // ! remove this later
            sleep(10);                                               // ! remove this later
        }
        if (nh.param("trigger/waypoint", seq_flag) == true){
            nh.setParam("trigger/waypoint", false);
            nh.setParam("trigger/metadata", false);                 // ! remove this later      
            ROS_INFO("Updating WP table.");
            waypoint_clear_client();                                // ! remove this later
            n = n + 2;
            if (n <= altitude.size() - 2){
                wp = create_waypoint(22, 0, latitude[n], longitude[n], altitude[n]);
                wp_push_srv.request.waypoints.push_back(wp);
                wp = create_waypoint(115, param1[n+1], latitude[n], longitude[n], altitude[n]);
                wp_push_srv.request.waypoints.push_back(wp);
                wp = create_waypoint(16, 0, latitude[n], longitude[n], altitude[n]);
                wp_push_srv.request.waypoints.push_back(wp);
                wp_client.call(wp_push_srv);
                change_mode();
            }
            else {
                ROS_INFO("End of WP table reached. Doing an RTL now.");
                wp = create_waypoint(20, 0, 0, 0, 0);               // ! double check this in SITL
                wp_push_srv.request.waypoints.push_back(wp);
                wp_client.call(wp_push_srv);
                change_mode();
            }
        }
    }
    return 0;
}