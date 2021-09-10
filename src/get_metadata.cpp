/* 
* C++ version of get_metadata.py
* When a WP is reached a flag is set ("trigger/sequence") which begins calibration from the 
* cal_sequence script. That script also sets the trigger/metadata flag which causes this 
* program to begin saving metadata from the FCU and SDR. 

* author: KM
* date: 28th Aug 2021
*/

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <fstream> 
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>
#include <iomanip>          // std::setprecision()
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/PositionTarget.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"

// Define global variables
double refresh_rate = 20.0;         // sensor refresh rate set by mavros (see mission.sh)
bool seq_flag;                      // sequene flag initialization b/c I do not understand C++
bool met_flag;                      // metadata flag initialization b/c I do not understand C++
struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;   // get path to home directory
std::string local_pose;             
std::string global_pose;            // these will point to the metadata filenames
std::string sdr_d_temp;
std::ofstream local_pose_f;
std::ofstream global_pose_f;        // declare ofsteam objects for writing to files later
std::ofstream sdr_d_temp_f;         //      using the callback sub functions

//? get current time and format for timestamping metadata
std::string get_timestamp(){
    struct timeval tmnow;
    struct tm *tm;
    char timestamp[30], usec_time[12], day[30];         // declare empty strings
    gettimeofday(&tmnow, NULL);                         // get time
    tm = localtime(&tmnow.tv_sec);                      // convert to readable time
    strftime(timestamp, 30, "%H:%M:%S", tm);            // format time upto sec resolution
    strcat(timestamp, ".");                             // add decimal point
    sprintf(usec_time, "%d", (int)tmnow.tv_usec);       // get time in ms
    strcat(timestamp, usec_time);                       // add time in ms res to time string
    strftime(day, 30, "-%d/%m/%Y", tm);                 // repeat the above to
    strcat(timestamp, day);                             //   have day info added as well
    return timestamp;
}

//? create a different timestamp for metadata filenames
std::string get_filename(){
    struct timeval tmnow;
    struct tm *tm;
    char namestamp[45];                                 // declare empty strings
    gettimeofday(&tmnow, NULL);                         // get time
    tm = localtime(&tmnow.tv_sec);                      // convert to readable time
    strftime(namestamp, 45, "%d-%m-%Y_%H-%M-%S", tm);   // translate to convenient format
    return namestamp;
}

//? close files after writing each line. is there a better way to do it?
void close_files(){
    local_pose_f.close();
    global_pose_f.close();
    sdr_d_temp_f.close();
}

//? define callback functions. these will write metadata to files.
//* IMU pose data
void callback_local(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::string current_time = get_timestamp();
    local_pose_f.open(local_pose, std::ios_base::app);
    local_pose_f << current_time << "\t" << msg->pose.position.x << "\t" 
                    << msg->pose.position.y << "\t" << msg->pose.position.z 
                    << std::setprecision(12) << std::endl;
}

//* GPS raw data
void callback_global(const sensor_msgs::NavSatFix::ConstPtr& msg){
    std::string current_time = get_timestamp();
    global_pose_f.open(global_pose, std::ios_base::app);
    global_pose_f << current_time << "\t" << msg->latitude << "\t" 
                    << msg->longitude << "\t" << msg->altitude  
                    << std::setprecision(12) << std::endl; 
}

//* SDR temperature
void callback_SDR(const std_msgs::Float32::ConstPtr& msg){
    std::string current_time = get_timestamp();
    sdr_d_temp_f.open(sdr_d_temp, std::ios_base::app);
    sdr_d_temp_f << current_time << "\t" << msg->data 
                    << std::setprecision(12) << std::endl;
}

int main(int argc, char **argv){
    //? create metadata files and add column headings
    std::string time_now = get_filename();
    std::string logs_path = "/catkin_ws/src/Drone-Project-code/logs/metadata/";
    logs_path = homedir + logs_path;
    std::string &local_ptr = local_pose;
    local_ptr = logs_path + time_now + "_local_pose.log";
    local_pose_f.open(local_pose);
    local_pose_f << "Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\n";
    std::string &global_ptr = global_pose;
    global_ptr = logs_path + time_now + "_global_pose.log";
    global_pose_f.open(global_pose);
    global_pose_f << "Timestamp\tLongitude\tLatitude\tAltitude\n";
    std::string &sdr_d_ptr = sdr_d_temp;
    sdr_d_ptr = logs_path + time_now + "_sdr_d_temp.log";
    sdr_d_temp_f.open(sdr_d_temp);
    sdr_d_temp_f << "Timestamp\tSDR Temperature (deg C)\n";
    close_files();

    //? initiate ROS node
    ros::init(argc, argv, "get_metadata");
    ros::NodeHandle n;
    n.setParam("trigger/metadata", false);
    int wp_num = 0;                                                 // WP number counter
    ROS_INFO("ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.");
    ros::Rate r(refresh_rate);
    while (ros::ok()) {
        usleep(1/refresh_rate * 1e6);                               // refresh rate in us
        if (n.param("trigger/metadata", met_flag) == true){         // check if flag set by wp_trigger
            ROS_INFO("Saving Waypoint #%i metadata in ./logs/metadata/", wp_num);
            local_pose_f.open(local_pose, std::ios_base::app);
            global_pose_f.open(global_pose, std::ios_base::app);    // append data to previous
            sdr_d_temp_f.open(sdr_d_temp, std::ios_base::app);      //      ofstream objects
            local_pose_f << "Waypoint #" << wp_num << std::endl;
            global_pose_f << "Waypoint #" << wp_num << std::endl;   // tag new waypoints
            sdr_d_temp_f << "Waypoint #" << wp_num << std::endl;
            close_files();
            wp_num++;
            ros::Subscriber loc_sub = n.subscribe("/mavros/local_position/pose", 1000, callback_local);
            ros::Subscriber glo_sub = n.subscribe("/mavros/global_position/global", 1000, callback_global);
            ros::Subscriber sdr_sub = n.subscribe("/sdr_temperature", 1000, callback_SDR);
            while (ros::ok()){
                ros::spinOnce();                                    // spin once because subscribe                                 
                r.sleep();                                          //   and spin work differently
                close_files();                                      //    in C++ and Python
                if (n.param("trigger/metadata", met_flag) == false){
                    ROS_INFO("Finished saving metadata for this WP.");
                    close_files();
                    loc_sub.shutdown();
                    glo_sub.shutdown();
                    sdr_sub.shutdown();
                    break;
                }
            }
        }
    }
    return 0;
}

