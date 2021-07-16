#!/usr/bin/env python

"""
This code will load up WPs from a target .waypoints file. It will push the first WP to the FC. 
When a calibration sequence has completed it will update the WP table on the FC.
If the list of WPs has run out (IndexError), it will force an RTL.
NOTE: To begin a new mission with the payload computer running, implement a while loop which 
will keep trying to push the first WPs to the FCU every few seconds? Make sure this does not 
happen when the drone is actually in a mission! Either that, or implemement a manual control 
for updating the .waypoints file.

author: Krishna Makhija
rev: 25th April 2021
"""
#FIXME: change the path of the lookup table.

import rospy, time
import csv
from os.path import expanduser
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *


#filename = "120m_10s_1passes_Ardupilot_4-wps.waypoints"
#filename    = "60m_10s_1passes_Ardupilot.waypoints"
filename    = "mission.waypoints"
path        = expanduser("~")  + "/catkin_ws/src/Drone-Project-code/mission/"
filename    = path + filename
rospy.set_param('trigger/waypoint', False)
rospy.set_param('trigger/sequence', False)
rospy.set_param('trigger/lock', False)          # block wp_trigger at each WP
timeout         = 12                            # timeout before updating new WP
lock_timeout    = 5                             # time to unlock trigger/sequence


def waypoint_clear_client():
    """
    Clear WPs from the FC.
    """
    try:
        response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        return response.call().success
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return False


def push_wp():
    """
    Push WP to the FC.
    """
    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush, persistent=True)
        service(start_index=0, waypoints=wl)
        if service.call(start_index=0, waypoints=wl).success:
            print("WP table updated %s" %wl)
        else:
            print("Write mission error")
    except rospy.ServiceException, e:
        print("Service call failed: %s" % e)


def change_mode():
    """
    Toggle modes before departing each WP. 
    The WP table needs to be cleared before updating with a new WP and switching 
    "back" to Auto.
    """
    try:
        service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        service(custom_mode="BRAKE")
        service(custom_mode="AUTO")
    except rospy.ServiceException, e:
        print("Set current WP failed: %s" % e)


def create_waypoint(command, param1, latitude ,longitude, altitude):
    """
    Create a waypoint object to be sent to the FC.
    Command: tells the drone what to do (navigate, land, RTL, etc)
        22: takeoff
        115: yaw
        16: navigate to WP
        19: loiter
        20: RTL
    param1: how to do it (delay at WP, yaw angle)
    """
    wp = Waypoint()
    wp.frame = 3
    wp.command = command  #takeoff
    wp.is_current = False
    wp.autocontinue = True
    wp.param1 = param1
    wp.param2 = 0
    wp.param3 = 0
    wp.param4 = 0
    wp.x_lat = latitude
    wp.y_long = longitude
    wp.z_alt = altitude
    return wp


def main():
    """
    Read WP table and update when conditions are met.
    """
    global wl
    n = 2
    with open(filename, 'r') as f:
        reader = csv.reader(f, dialect='excel', delimiter='\t')
        reader = list(reader)
    waypoint_clear_client()
    wl = []
    wp = create_waypoint(22, 0, float(reader[1][8]), float(reader[1][9]), float(reader[1][10]))
    wl.append(wp)
    wp = create_waypoint(115, float(reader[n+1][4]), float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
    wl.append(wp)
    wp = create_waypoint(16, 0, float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
    wl.append(wp)
    push_wp()
    while True:
        time.sleep(0.05)
        # * clear waypoint table before ending sequence so that it does not re-trigger.
        if rospy.get_param('trigger/sequence') == True:
            waypoint_clear_client()
            rospy.set_param('trigger/sequence', False)
        if rospy.get_param('trigger/waypoint') == True:
            rospy.set_param('trigger/waypoint', False)
            try:
                print("Updating WP table.")
                n = n + 2
#                waypoint_clear_client()
                wl = []
                wp = create_waypoint(22, 0, float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
                wl.append(wp)
                wp = create_waypoint(115, float(reader[n+1][4]), float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
                wl.append(wp)
                wp = create_waypoint(16, 0, float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
                wl.append(wp)
                push_wp()
                change_mode()
            except IndexError:
                print("End of WP table reached. Doing an RTL now.")
                try:
                    #FIXME: the coords here should be different because this is triggering the sequence
                    wp = create_waypoint(20, 0, float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
                    wl.append(wp)
                    push_wp()
                    change_mode()
                except IndexError:
                    pass
            time.sleep(lock_timeout)
            rospy.set_param('trigger/lock', False)

if __name__ == '__main__':
    main()

