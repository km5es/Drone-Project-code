#!/usr/bin/env python2
"""
Rewrite Varundev's ROS code so that it does not look for a csv file instead just sets a trigger when 
a waypoint is reached.

Author: KM
Date: 07/01/2021
modified: 27/01/2021
"""
import rospy, time, re
import numpy as np
from termcolor import colored
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import WaypointReached, WaypointList, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from threading import Event


rospy.set_param('trigger/command', False)
rospy.set_param('trigger/acknowledgement', False)
event = Event()

error_tolerance = 1.0       ## distance in m from where to begin sequence
sleep_time      = 15

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
    wp_list = data.waypoints
    # skip first two waypoints, i.e. home and takeoff
    wp_list = wp_list[2::2]
    #file = open("test_wp.txt", "w+")
    wp_x_lat = []
    wp_y_long = []
    wp_z_alt = []
    # m[17] is lat 19 is long and 21 is alt
    # skip final waypoint, i.e. land
    for n in range(len(wp_list)-1):
        m = re.split("\s", str(wp_list[n]))
        wp_x_lat.append(float(m[17]))
        wp_y_long.append(float(m[19]))
        wp_z_alt.append(float(m[21]))
        #file.write(str(m[17]))
        #file.write("\t")
        #file.write(str(m[19]))
        #file.write("\t")
        #file.write(str(m[21]))
        #file.write("\n")


def get_haversine(data):
    """
    Calculate haversine distance to target using real-time GPS data
    """
    global h
    h = []
    if event.is_set() == False:
        for n in range(len(wp_x_lat)):
            rospy.sleep(1e-6)
            h1 = haversine(data.latitude, data.longitude, wp_x_lat[n], wp_y_long[n])
            h.append(h1)
    event.set()


def get_distance(data):
    """
    Calculate 3D distance to target
    """
    global distance
    distance = []
    if event.is_set():
        try:
            for n in range(len(wp_x_lat)):
                rospy.sleep(1e-6)
                alt_diff = wp_z_alt[n] - data.pose.position.z
                distance.append((h[n]**2 + alt_diff**2)**0.5)
        except IndexError:
            pass
        for i in distance:
            print('The closest WP is: ' +str(min(distance)) + 'm away')
            if i <= error_tolerance:
                print(">>>>WP reached<<< ||| Pausing script for " +str(sleep_time) + " seconds")
                rospy.set_param('trigger/command', True)
                time.sleep(sleep_time)
    event.clear()


def main():
    try:
        rospy.init_node('ground_station', anonymous = True)
    #        rospy.Subscriber('/mavros/mission/reached', WaypointReached, trigger_node)
        rospy.Subscriber('/mavros/mission/waypoints', WaypointList, get_waypoints)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, get_haversine)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, get_distance)
        rospy.spin()
    except (rospy.ROSInterruptException):
        pass


if __name__ == '__main__':
    main()
