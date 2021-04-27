#!/usr/bin/env python

"""
This code will trigger when a WP is reached and when the linear velocity is below vel_threshold.
It will trigger a flag which in turn will begin the cal sequence at each WP. When sequence is 
completed, another flag will trigger on write_WPs.py which will update the WP table. 
This node will also save metadata.

author: Krishna Makhija
rev: 25th April 2021
"""
#FIXME: isntead of updating wp_seq, maybe there is a way to check explicitly if cond yaw is met?
#FIXME: use that instead to set wp event?

import rospy, time
import numpy as np
from termcolor import colored
from threading import Event, Thread
from os.path import expanduser
from std_msgs.msg import String
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointReached, WaypointList, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry


n               = 1
event           = Event()
vel_threshold   = 0.15      # linear vel threshold below which drone is considered "stationary" (m/s)
wp_num          = 1
rospy.set_param('trigger/sequence', False)

def get_velocity(data):
    """
    Get the current magnitude of linear velocity of the drone.
    """
    global v
    x_vel = data.twist.twist.linear.x
    y_vel = data.twist.twist.linear.y
    z_vel = data.twist.twist.linear.z
    v = (x_vel**2 + y_vel**2 + z_vel**2)**0.5
    #print(x_vel, y_vel, z_vel)
    #print("The current velocity is %s m/s" %v)
    if event.is_set() and v < vel_threshold:
        print("Triggering  payload flag")
        #rospy.set_param('trigger/waypoint', True)
        rospy.set_param('trigger/sequence', True)
        event.clear()


def wp_reached(data):
    """
    Set trigger when arrived at WP.
    """
    global n
    global sequence
    global wp_num
    if n == 1:
        sequence = data.header.seq
        n = 2
        print("The sequence value is set at %s" %sequence)
    print("The current drone sequence on the FCU is %s" %data.header.seq)
    if data.header.seq == sequence + 2:
        print("WP reached: %s" %wp_num)
        wp_num = wp_num + 1
        event.set()
        sequence = data.header.seq


def main():
    try:
        rospy.init_node('wp_trigger', anonymous = True)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, wp_reached)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, get_velocity)
        rospy.spin()
    except (rospy.ROSInterruptException):
        pass


if __name__ == '__main__':
    main()