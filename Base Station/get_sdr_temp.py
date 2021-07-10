"""
This ROS node will run on the base station during data acquisition. Its purpose is 
to save SDR temperature data during waypoints. This is probably not the most efficient 
way to save SDR temperature but will do for now.

Author: Krishna Makhija
date: Jun 29th 2021
"""

import rospy, time, re
import numpy as np
from os.path import expanduser
from termcolor import colored
from std_msgs.msg import Float32


path            = expanduser("~")             # data files save path
logs_path       = path + '/catkin_ws/src/Drone-Project-code/logs/metadata/'             
sdr_ground_temp = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_sdr_ground_temp.log")
refresh_rate    = 10.0


def get_timestamp():
    """
    Returns current time for data logging.
    """
    time_now = time.time()
    mlsec = repr(time_now).split('.')[1][:3]
    time_now = time.strftime("%H:%M:%S.{}-%d/%m/%Y".format(mlsec))
    return time_now


def callback_SDR(data):
    """
    Callback object for SDR temperature.
    """
    try:
        current_time = get_timestamp()
        sdr_ground_temp_f_a.write("%s\t%s\n" %(current_time, data.data))
        rospy.sleep(1/refresh_rate)
    except ValueError, NameError:
        pass


def main():
    """
    main function for acquiring metadata at waypoints.
    """
    global sdr_ground_temp_f_a

    sdr_ground_temp_f = open(sdr_ground_temp, "w+")
    sdr_ground_temp_f.write("Timestamp\tSDR Temperature (deg C)\n")
    sdr_ground_temp_f.close()

    rospy.init_node('get_sdr_temp', anonymous=True)
    rospy.set_param('trigger/metadata', False)
    wp_num = 0
    print(colored('ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.', 'green'))
    while not rospy.is_shutdown():
        time.sleep(0.01)
        if rospy.get_param('trigger/metadata') == True:
            current_time = time.strftime("%H%M%S-%d%m%Y")
            print(colored('[' + str(current_time) + ']Saving Waypoint #' + str(wp_num) + ' metadata in ' +
                          str(sdr_ground_temp), 'grey', 'on_white'))
            sdr_ground_temp_f_a = open(sdr_ground_temp, "a+")
            sdr_ground_temp_f_a.write("Waypoint #%s\n" % (wp_num))
            wp_num += 1
            rospy.Subscriber('/sdr_temperature', Float32, callback_SDR)
            while True:
                if rospy.get_param('trigger/metadata') == False:
                    print('Finished saving metadata for this WP.')
                    sdr_ground_temp_f_a.close()
                    break


if __name__ == '__main__':
    main()
