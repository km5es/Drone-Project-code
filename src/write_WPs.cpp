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
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <pwd.h>

struct passwd *pw = getpwuid(getuid());
const char *homedir = pw->pw_dir;   // get path to home directory




int main(){
    std::cout << homedir << std::endl;
    return 0;
}
