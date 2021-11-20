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
from threading import Event
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointReached, WaypointList, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion


n               = 1
event           = Event()
vel_threshold   = 0.2      # linear vel threshold below which drone is considered "stationary" (m/s)
wp_num          = 1
#rospy.set_param('trigger/waypoint', False)
rospy.set_param('trigger/sequence', False)
seq_timeout     = 30
error_tolerance = 1.0       ## distance in m from where to begin sequence
orien_tolerance = 10.0      ## how much pitch and roll per wp is allowed (deg)
wp_wait_timeout = 10


def get_velocity(data):
    """
    Get the current magnitude of linear velocity of the drone.
    """
    global v
    global roll, pitch
    x_vel = data.twist.twist.linear.x
    y_vel = data.twist.twist.linear.y
    z_vel = data.twist.twist.linear.z
    v = (x_vel**2 + y_vel**2 + z_vel**2)**0.5       # magnitude of linear velocity
    q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,    # angular orientation
                data.pose.pose.orientation.z, data.pose.pose.orientation.w] # in quarternion form
    euler_angs = np.rad2deg(euler_from_quaternion(q, axes='sxyz'))
    roll = euler_angs[0]
    pitch = euler_angs[1]


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
    #time.sleep(0.01)
    if data.status.status == 0:
        if event.is_set() == False:
            h = haversine(data.latitude, data.longitude, wp_x_lat, wp_y_long)
        event.set()
    elif data.status.status == -1:
        print('GPS fix not available.')


def get_distance(data):
    """
    Calculate 3D haversine distance to target.
    This will also begin the entire sequence.
    """
    global distance
    if event.is_set():
        try:
            alt_diff = wp_z_alt - data.pose.position.z
            distance = (h**2 + alt_diff**2)**0.5
            #print('The closest WP is: %s m away.' %(distance))
            event.clear()
            ## * check to see if drone is stable enough to start calibration
            if distance <= error_tolerance and v <= vel_threshold and (roll <= orien_tolerance and pitch <= orien_tolerance):
                print(">>>>WP reached<<< ||| Drone is stable and (almost) not moving.")
                #rospy.set_param('trigger/waypoint', True)
                time.sleep(2)
                rospy.set_param('trigger/sequence', True)
                #FIXME: this is another open loop. what do? can't seem to avoid them
                time.sleep(wp_wait_timeout)
        except IndexError:
            print("index error")
            pass
        except NameError:
            print("Waypoints not received from FCU.")
            pass


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