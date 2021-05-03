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

import rospy, time, re
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
error_tolerance = 1.0       ## distance in m from where to begin sequence
GPS_refresh     = 10


def get_velocity(data):
    """
    Get the current magnitude of linear velocity of the drone.
    """
    global v

    x_vel = data.twist.twist.linear.x
    y_vel = data.twist.twist.linear.y
    z_vel = data.twist.twist.linear.z
    v = (x_vel**2 + y_vel**2 + z_vel**2)**0.5

#    try:
#        if event.is_set() and v < vel_threshold:
#            print("Drone is (almost) not moving. Triggering  payload flag")
#            #rospy.set_param('trigger/waypoint', True)
#            rospy.set_param('trigger/sequence', True)
#            event.clear()
#        elif timeout_event.is_set():
#            start = time.time()
#            timeout_event.clear()
#        if time.time() >= start + seq_timeout:
#            #FIXME: currently, this runs on a loop after final waypoint. fix?
#            print("Seq timeout. Updating WP table")
#            rospy.set_param('trigger/waypoint', True)
#            start = time.time()
#    except UnboundLocalError, NameError:
#        pass


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


def haversine(lat1, long1, lat2, long2):
    """
    Calculate distance between two points given lat/long coordinates.
    REFERENCE: https://www.movable-type.co.uk/scripts/latlong.html
    """
    r = 6.3781e6                # radius of earth
    phi1      = np.deg2rad(lat1)
    phi2      = np.deg2rad(lat2)
    lam1      = np.deg2rad(long1)
    lam2      = np.deg2rad(long2)
    delta_phi = phi2 - phi1
    delta_lam = lam2 - lam1
    a         = np.sin(delta_phi/2) * np.sin(delta_phi/2) + np.cos(lat1) * np.cos(lat2) * np.sin(delta_lam/2) * np.sin(delta_lam/2)
    c         = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d         = r * c           # distance
    return d


def haversine_3d(lat1, long1, alt1, lat2, long2, alt2):
    """
    Calculate haversine distance in 3 dimensions i.e. Euclidean distance using lat / long coordinates
    """
    alt_diff = (alt2 - alt1)
    d_3d = ((haversine(lat1, long1, lat2, long2))**2 + alt_diff**2)**0.5
    return d_3d


def get_waypoints(data):
    """
    Look up waypoints in FC and "target" them.
    """
    global wp_x_lat
    global wp_y_long
    global wp_z_alt
    try:
        wp_list = data.waypoints
        # skip first two waypoints, i.e. home and takeoff
        target_wp = wp_list[2:]
        wp_x_lat = target_wp[0].x_lat
        wp_y_long = target_wp[0].y_long
        wp_z_alt = target_wp[0].z_alt
        print("Retrieved WP list.")
        print("The current target WP coords are: %s, %s, and %s" %(wp_x_lat, wp_y_long, wp_z_alt))
    except IndexError:
        pass


def get_haversine(data):
    """
    Calculate 2D haversine distance to target using real-time GPS data
    """
    global h
    #while True:
    time.sleep(0.01)
    if data.status.status == 0:
        if event.is_set() == False:
            h = haversine(data.latitude, data.longitude, wp_x_lat, wp_y_long)
        event.set()
    elif data.status.status == -1:
        print('GPS fix not available.')


def get_distance(data):
    """
    Calculate 3D haversine distance to target
    """
    global distance
    if event.is_set():
        try:
            alt_diff = wp_z_alt - data.pose.position.z
            distance = (h**2 + alt_diff**2)**0.5
        #except (IndexError, NameError):
        except IndexError:
            print("index error")
            pass
        except NameError:
            print("Waypoints not received from FCU.")
            pass
        print('The closest WP is: %s m away.' %(distance))
        if distance <= error_tolerance and v <= vel_threshold and rospy.get_param('trigger/waypoint') == False:
            print(">>>>WP reached<<< ||| Drone is stable and (almost) not moving.")
            #rospy.set_param('trigger/waypoint', True)
            rospy.set_param('trigger/sequence', True)
    event.clear()


def main():
    global get_mission
    try:
        rospy.init_node('wp_trigger', anonymous = True)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, get_waypoints)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, get_haversine)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, get_distance)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, get_velocity)
        rospy.spin()
    except (rospy.ROSInterruptException):
        pass


def main_seq():
    try:
        rospy.init_node('wp_trigger', anonymous = True)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, wp_reached)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, get_velocity)
        rospy.spin()
    except (rospy.ROSInterruptException):
        pass


if __name__ == '__main__':
    main()