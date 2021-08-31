/* 
* C++ version of get_metadata.py
* When a WP is reached a flag is set ("trigger/sequence"). This will begin saving metadata
* in separate files in the ./logs folder. It will also set another flag ("trigger/metadata")
* which will begin calibration.

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
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/PositionTarget.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"

double refresh_rate = 10.0;         // sensor refresh rate set by mavros (see mission.sh)
bool seq_flag;                      // sequene flag initialization b/c I do not understand C++

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

//? create a different timestamp for metadata filenames and pass to make_file()
std::string get_filename(){
    struct timeval tmnow;
    struct tm *tm;
    char namestamp[45];                                 // declare empty strings
    gettimeofday(&tmnow, NULL);                         // get time
    tm = localtime(&tmnow.tv_sec);                      // convert to readable time
    strftime(namestamp, 45, "%d-%m-%Y_%H-%M-%S", tm);   // translate to convenient format
    return namestamp;
}

//? create metadata files and add column headings
void make_file(){
    std::string time_now = get_filename();
    std::string logs_path = "/home/kmakhija/catkin_ws/src/Drone-Project-code/logs/metadata/";
    std::string local_pose = logs_path + time_now + "_local_pose.log";
    std::ofstream local_pose_f;
    local_pose_f.open(local_pose);
    local_pose_f << "Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\n" << std::endl;
    local_pose_f.close();
    std::string global_pose = logs_path + time_now + "_global_pose.log";
    std::ofstream global_pose_f;
    global_pose_f.open(global_pose);
    global_pose_f << "Timestamp\tLongitude\tLatitude\tAltitude\n";
    global_pose_f.close();
    std::string sdr_d_temp = logs_path + time_now + "_sdr_drone_temp.log";
    std::ofstream sdr_d_temp_f;
    sdr_d_temp_f.open(sdr_d_temp);
    sdr_d_temp_f << "Timestamp\tSDR Temperature (deg C)";
    sdr_d_temp_f.close();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "get_metadata");
    ros::NodeHandle n;
    n.setParam("trigger/metadata", false);
    make_file();
    while (ros::ok()){
        sleep(1);
        if (n.param("trigger/sequence", seq_flag) == true){
            std::string time_now = get_timestamp();
            std::cout << "The time now is: " << time_now << std::endl;
        }
    }
    return 0;
}

