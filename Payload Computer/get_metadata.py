"""
This script will run on startup on the payload computer. When a flag is set through ROS, it will begin 
saving metadata from the USB connection. Note that it requires MAVROS to be running in parallel to work.

Author: Krishna Makhija
date: Sep 8th 2020
"""
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from threading import Event, Thread
import time
from termcolor import colored
from std_msgs.msg import Float32

path        = '/home/ubuntu/'             # data files save path
metadata    = path + 'meta.dat'
file        = open(metadata, 'w')
acq_event   = Event()
timeout     = 4

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


def listener():
    """
    ROS listener node for IMU data. This should go on the payload computer. Set a ros param through the
    payload code when cal begins and ends. Another node listens for that and initiates and disables meta
    data generation. Why do it this way? Because you can't multithread this for some reason.
    """
    file.write('Timestamp\tTemperature\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\tSetpoint (x)\tSetpoint (y)\tSetpoint (z)\n')
    rospy.init_node('get_metadata', anonymous=True)
    rospy.set_param('trigger/metadata', False)
    while not rospy.is_shutdown():
        time.sleep(0.001)
        if rospy.get_param('trigger/metadata') == True:
            print(colored('Saving metadata in ' + str(metadata), 'grey', 'on_white'))
            rospy.Subscriber('/mavros/local_position/pose',
                             PoseStamped, callback_local)
            rospy.Subscriber('/mavros/setpoint_raw/target_local',
                             PositionTarget, callback_setpoint)
            rospy.Subscriber('sdr_temperature', Float32, callback_SDR)
            rospy.spin()
            if rospy.get_param('trigger/metadata') == False:
                print('Finished saving metadata for this WP.')
                break


if __name__ == '__main__':
    listener()