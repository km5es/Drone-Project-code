"""
This script will run on startup on the payload computer. When a flag is set through ROS, it will begin 
saving metadata from the USB connection. Note that it requires MAVROS to be running in parallel to work.
Note also that data from each ROS topic is saved in a separate file.

Author: Krishna Makhija
date: Sep 8th 2020
"""

import rospy, time, re
import numpy as np
from os.path import expanduser
from termcolor import colored
from std_msgs.msg import Float32
from threading import Event, Thread
from geometry_msgs.msg import PoseStamped
from termcolor import colored
from mavros_msgs.msg import WaypointReached, WaypointList, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from threading import Event
from std_msgs.msg import String
from mavros_msgs.msg import *
from mavros_msgs.srv import *


path            = expanduser("~") + "/"             # data files save path
logs_path       = path + '/catkin_ws/src/Drone-Project-code/logs/metadata/'             
local_pose      = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_local_pose.log")
global_pos      = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_global_pose.log")
set_target      = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_set_target.log")
sdr_drone_temp  = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_sdr_drone_temp.log")
refresh_rate    = 10.0


def get_timestamp():
    """
    Returns current time for data logging.
    """
    time_now = time.time()
    mlsec = repr(time_now).split('.')[1][:3]
    time_now = time.strftime("%H:%M:%S.{}-%d/%m/%Y".format(mlsec))
    return time_now


def callback_local(data):
    """
    Callback object for drone's local position and timestamp.
    """
    try:
        current_time = get_timestamp()
        local_pose_f_a.write("%s\t%s\t%s\t%s\n" % (current_time, data.pose.position.x,
                                                   data.pose.position.y, data.pose.position.z))
        rospy.sleep(1/refresh_rate)
    except ValueError:
        pass


def callback_setpoint(data):
    """
    Callback object for drone's setpoint position and timestamp.
    """
    try:
        current_time = get_timestamp()
        set_target_f_a.write("%s\t%s\t%s\t%s\n" %
                             (current_time, data.position.x, data.position.y, data.position.z))
        rospy.sleep(1/refresh_rate)
    except ValueError:
        pass


def callback_global(data):
    """
    Callback object for drone's GPS location.
    """
    try:
        current_time = get_timestamp()
        global_pos_f_a.write("%s\t%s\t%s\t%s\n" %(current_time, data.latitude, data.longitude, data.altitude))
        rospy.sleep(1/refresh_rate)
    except ValueError:
        pass


def callback_SDR(data):
    """
    Callback object for SDR temperature.
    """
    try:
        current_time = get_timestamp()
        sdr_drone_temp_f_a.write("%s\t%s\n" %(current_time, data.data))
        rospy.sleep(1/refresh_rate)
    except ValueError, NameError:
        pass


def main_sitl():
    """
    Initiate metadata file and write to it. Apparently, this only works with the SITL sims. Go figure.
    """
    rospy.init_node('get_metadata', anonymous=True)
    rospy.set_param('trigger/metadata', False)
    file = open(metadata, "w+")
    file.write("Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\tSet_target (x)\tSetpoint (y)\tSetpoint (z)\tTemperature\n")
    file.close()
    wp_num = 0
    print(colored('ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.', 'green'))
    while not rospy.is_shutdown():
        time.sleep(0.001)
        if rospy.get_param('trigger/metadata') == True:
            print(colored('Saving Waypoint #' + str(wp_num) +
                          ' metadata in ' + str(metadata), 'grey', 'on_white'))
            file = open(metadata, "a+")
            file.write("Waypoint #%s\n" % (wp_num))
            wp_num += 1
            while True:
                current_time = time.strftime("%H%M%S-%d%m%Y")
                print(current_time)
                local_pose = rospy.wait_for_message(
                    '/mavros/local_position/pose', PoseStamped)
                set_target = rospy.wait_for_message(
                    '/mavros/setpoint_raw/target_local', PositionTarget)
                sdr_temp = rospy.wait_for_message('sdr_temperature', Float32)
                file.write("%s\t%s\t%s\t%s\t" % (current_time, local_pose.pose.position.x,
                                                 local_pose.pose.position.y, local_pose.pose.position.z))
                file.write("%s\t%s\t%s\n" % (set_target.position.x,
                                             set_target.position.y, set_target.position.z))

                if rospy.get_param('trigger/metadata') == False:
                    print('Finished saving metadata for this WP.')
                    file.close()
                    break


def main():
    """
    main function for acquiring metadata at waypoints.
    """
    global local_pose_f_a
    global set_target_f_a
    global global_pos_f_a
    global sdr_drone_temp_f_a

    local_pose_f = open(local_pose, "w+")
    local_pose_f.write(
        "Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\n")
    local_pose_f.close()
    set_target_f = open(set_target, "w+")
    set_target_f.write(
        "Timestamp\tSet_target (x)\tSet_target (y)\tSet_target (z)\n")
    set_target_f.close()
    global_pos_f = open(global_pos, "w+")
    global_pos_f.write("Timestamp\tLatitude\tLongitude\tAltitude\n")
    global_pos_f.close()
    sdr_drone_temp_f = open(sdr_drone_temp, "w+")
    sdr_drone_temp_f.write("Timestamp\tSDR Temperature (deg C)")
    sdr_drone_temp_f.close()

    rospy.init_node('get_metadata', anonymous=True)
    rospy.set_param('trigger/metadata', False)
    wp_num = 0
    print(colored('ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.', 'green'))
    while not rospy.is_shutdown():
        time.sleep(0.01)
        if rospy.get_param('trigger/metadata') == True:
            current_time = time.strftime("%H%M%S-%d%m%Y")
            print(colored('[' + str(current_time) + ']Saving Waypoint #' + str(wp_num) + ' metadata in ' +
                          str(local_pose) + ' and ' + str(set_target), 'grey', 'on_white'))
            local_pose_f_a = open(local_pose, "a+")
            set_target_f_a = open(set_target, "a+")
            global_pos_f_a = open(global_pos, "a+")
            sdr_drone_temp_f_a = open(sdr_drone_temp, "a+")
            local_pose_f_a.write("Waypoint #%s\n" % (wp_num))
            set_target_f_a.write("Waypoint #%s\n" % (wp_num))
            global_pos_f_a.write("Waypoint #%s\n" % (wp_num))
            sdr_drone_temp_f_a.write("Waypoint #%s\n" % (wp_num))
            wp_num += 1
            rospy.Subscriber('/mavros/local_position/pose',
                                PoseStamped, callback_local)
            rospy.Subscriber('/mavros/setpoint_raw/target_local',
                                PositionTarget, callback_setpoint)
            rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback_global)
            rospy.Subscriber('/sdr_temperature', Float32, callback_SDR)
            while True:
                if rospy.get_param('trigger/metadata') == False:
                    print('Finished saving metadata for this WP.')
                    local_pose_f_a.close()
                    set_target_f_a.close()
                    global_pos_f_a.close()
                    sdr_drone_temp_f_a.close()
                    break


if __name__ == '__main__':
    main()
