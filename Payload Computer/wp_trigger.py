#!/usr/bin/env python

"""
This code will trigger when a WP is reached and when the linear velocity is below vel_threshold.
It will trigger a flag which in turn will begin the cal sequence at each WP which in turn will 
begin saving metadata. When sequence is completed, another flag will trigger on write_WPs.py 
which will update the WP table. 
If the sequence does not complete within a specified time, the WP table will be updated anyway.

author: Krishna Makhija
rev: 25th April 2021
"""

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
#rospy.set_param('trigger/waypoint', False)
rospy.set_param('trigger/sequence', False)
seq_timeout     = 30
timeout_event   = Event()
start           = time.time()


def get_velocity(data):
    """
    Get the current magnitude of linear velocity of the drone.
    """
    global v
    global start

    x_vel = data.twist.twist.linear.x
    y_vel = data.twist.twist.linear.y
    z_vel = data.twist.twist.linear.z
    v = (x_vel**2 + y_vel**2 + z_vel**2)**0.5

    try:
        if event.is_set() and v < vel_threshold:
            print("Drone is (almost) not moving. Triggering  payload flag")
            #rospy.set_param('trigger/waypoint', True)
            rospy.set_param('trigger/sequence', True)
            event.clear()
        elif timeout_event.is_set():
            start = time.time()
            timeout_event.clear()
        if time.time() >= start + seq_timeout:
            #FIXME: currently, this runs on a loop after final waypoint. fix?
            print("Seq timeout. Updating WP table")
            rospy.set_param('trigger/waypoint', True)
            start = time.time()
    except UnboundLocalError, NameError:
        pass


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
    if data.header.seq == sequence + 1:
        print("Begin countdown to updating WP table: %s seconds." %seq_timeout)
        timeout_event.set()
    if data.header.seq == sequence + 2:        
        print("WP reached: %s. Waiting for drone to be stationary." %wp_num)
        wp_num = wp_num + 1
        event.set()
        timeout_event.clear()
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