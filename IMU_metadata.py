"""
Test acquiring metadata
Author: KM
Date: Aug 16th 2020
"""
# TODO: Add an end WP marker like a line or something?

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from threading import Event
import time
from termcolor import colored
from std_msgs.msg import Float32

path = '/home/kmakhija/'             # data files save path
metadata = path + 'meta.dat'
file = open(metadata, 'w')
acq_event = Event()
timeout = 4


def callback_local(data):
    """
    Callback object for drone's local position.
    """
    file.write("\t\t%s\t%s\t%s" % (data.pose.position.x,
                                   data.pose.position.y, data.pose.position.z))
    rospy.sleep(0.2)


def callback_setpoint(data):
    """
    Callback object for drone's  setpoint position.
    """
    file.write("\t\t\t\t\t%s\t%s\t%s\n" %
               (data.position.x, data.position.y, data.position.z))
    rospy.sleep(0.2)


def callback_SDR(data):
    """
    Callback object for SDR temperature.
    """
    current_time = time.strftime("%H%M%S-%d%m%Y")
    file.write("%s\t%s\t" % (current_time, data))
    rospy.sleep(0.2)


def get_metadata():
    """
    ROS listener node for IMU data.
    """
    file.write("Timestamp\tTemperature\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\tSetpoint (x)\tSetpoint (y)\tSetpoint (z)\n")
    rospy.init_node('get_metadata', anonymous=True)
    while True:
        if acq_event.is_set():
            print(colored('Saving metadata in ' +
                          str(metadata), 'grey', 'on_white'))
            start = time.time()
            start_timeout = start + timeout
            rospy.Subscriber('sdr_temperature', Float32, callback_SDR)
            rospy.Subscriber('/mavros/local_position/pose',
                             PoseStamped, callback_local)
            rospy.Subscriber('/mavros/setpoint_raw/target_local',
                             PositionTarget, callback_setpoint)
            rospy.spin()
            if acq_event.is_set() == False:
                break
            elif time.time() > start_timeout:
                break


def listener():
    """
    ROS listener node for IMU data.
    """
    file.write('Timestamp\tTemperature\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\tSetpoint (x)\tSetpoint (y)\tSetpoint (z)\n')
    rospy.init_node('get_metadata', anonymous=True)
    trigger = raw_input('Enter trigger here: ')
    if trigger == 'trigger':
        print(colored('Saving metadata in ' + str(metadata), 'grey', 'on_white'))
        rospy.Subscriber('/mavros/local_position/pose',
                         PoseStamped, callback_local)
        rospy.Subscriber('/mavros/setpoint_raw/target_local',
                         PositionTarget, callback_setpoint)
        rospy.Subscriber('sdr_temperature', Float32, callback_SDR)
        rospy.spin()


if __name__ == '__main__':
    listener()
