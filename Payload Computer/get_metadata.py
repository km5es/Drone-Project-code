"""
This script will run on startup on the payload computer. When a flag is set through ROS, it will begin 
saving metadata from the USB connection. Note that it requires MAVROS to be running in parallel to work.
Note also that data from each ROS topic is saved in a separate file.

Author: Krishna Makhija
date: Sep 8th 2020
"""

import rospy, time
from os.path import expanduser
from termcolor import colored
from std_msgs.msg import Float32
from threading import Event, Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

path            = expanduser("~") + "/"             # data files save path
local_pose      = path + 'local_pose_meta.dat'
set_target      = path + 'set_target_meta.dat'
refresh_rate    = 25.0


def callback_local(data):
    """
    Callback object for drone's local position and timestamp.
    """
    try:
        time.sleep(0.01)
        current_time = time.strftime("%H%M%S-%d%m%Y")
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
        time.sleep(0.01)
        current_time = time.strftime("%H%M%S-%d%m%Y")
        set_target_f_a.write("%s\t%s\t%s\t%s\n" %
                             (current_time, data.position.x, data.position.y, data.position.z))
        rospy.sleep(1/refresh_rate)
    except ValueError:
        pass


def callback_SDR(data):
    """
    Callback object for SDR temperature.
    """
    file.write("%s\t" % (data))
    rospy.sleep(1/refresh_rate)


def main():
    """
    main function for acquiring metadata at waypoints.
    """
    global local_pose_f_a
    global set_target_f_a

    local_pose_f = open(local_pose, "w+")
    local_pose_f.write(
        "Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\n")
    local_pose_f.close()
    set_target_f = open(set_target, "w+")
    set_target_f.write(
        "Timestamp\tSet_target (x)\tSet_target (y)\tSet_target (z)\n")
    set_target_f.close()
    rospy.init_node('get_metadata', anonymous=True)
    rospy.set_param('trigger/metadata', False)
    wp_num = 0
    print(colored('ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.', 'green'))
    while not rospy.is_shutdown():
        time.sleep(0.01)
        if rospy.get_param('trigger/metadata') == True:
            print(colored('Saving Waypoint #' + str(wp_num) + ' metadata in ' +
                          str(local_pose) + ' and ' + str(set_target), 'grey', 'on_white'))
            local_pose_f_a = open(local_pose, "a+")
            set_target_f_a = open(set_target, "a+")
            local_pose_f_a.write("Waypoint #%s\n" % (wp_num))
            set_target_f_a.write("Waypoint #%s\n" % (wp_num))
            wp_num += 1
            while True:
                rospy.Subscriber('/mavros/local_position/pose',
                                 PoseStamped, callback_local)
                rospy.Subscriber('/mavros/setpoint_raw/target_local',
                                 PositionTarget, callback_setpoint)
                if rospy.get_param('trigger/metadata') == False:
                    print('Finished saving metadata for this WP.')
                    local_pose_f_a.close()
                    set_target_f_a.close()
                    break


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


if __name__ == '__main__':
    main()
