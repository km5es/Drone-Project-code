/*
* C++ version of the Python program with the same name. 

* author: Krishna Makhija
* date: Sep 10th 2021
*/

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <fstream> 
#include <time.h>

bool seq_flag;

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

void stream_file(){
    ros::NodeHandle n;
    // stream zeros file
    if (n.param("trigger/sequence", seq_flag) == true){
        n.setParam("trigger/sequence", false);
        n.setParam("trigger/metadata", true);
        std::cout << get_timestamp() <<": WP reached." << std::endl;
        // stream file
    }
}

int main(int argc, char **argv){
    std::cout << get_timestamp() << std::endl;
    stream_file();
    return 0;
}