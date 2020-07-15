#!/usr/bin/env python

import rospy
import csv
import os
import atexit

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import WaypointReached
from geographiclib.geodesic import Geodesic

global count
global flag
global cur_wp_index
global trigger

count           = 0
flag            = False
cur_wp_index    = -1
trigger         = False
error_tolerance = 1
mission         = []

rospy.set_param('trigger/command', False)
rospy.set_param('trigger/acknowledgement', False)

position_info   = []
event_info      = []

# Begin walk around test (debug only!)

#mission = [[38.0320166, -78.5110083], [38.0321571, -78.5109306], [38.0322860, -78.5108581]]

# End walk around test

def get_mission():

    file_path = os.path.expanduser('~/catkin_ws/src/Drone-Nav/mission/waypoints.csv')

    with open(file_path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter = ',')
        for waypoint in csv_reader:
            mission.append(waypoint)

    info = []
    msg  = 'converting waypoints to correct format'

    info.append(rospy.Time.now())
    info.append(msg)
    rospy.loginfo(msg)
    event_info.append(info)

    for index in range(0, len(mission)):
        for point in range(0, len(mission[index])):
            mission[index][point] = float(mission[index][point])

    info = []
    msg  = 'successfully retrieved global plan'

    info.append(rospy.Time.now())
    info.append(msg)
    rospy.loginfo(msg)
    event_info.append(info)



def save_events():

    file_path = os.path.expanduser('~/catkin_ws/src/Drone-Nav/logs/mission_events.csv')

    with open(file_path, mode = 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter = ',', quoting = csv.QUOTE_NONNUMERIC)
        for index in range(0, len(event_info)):
            csv_writer.writerow([event_info[index][0],
                                 event_info[index][1]])

def save_poses():

    file_path = os.path.expanduser('~/catkin_ws/src/Drone-Nav/logs/pose_logs.csv')

    with open(file_path, mode = 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter = ',', quoting = csv.QUOTE_NONNUMERIC)
        for index in range(0, len(position_info)):
            csv_writer.writerow([position_info[index][0],
                                 position_info[index][1],
                                 position_info[index][2],
                                 position_info[index][3]])

def save_data():

    info = []
    msg = 'shutdown signal detected'
    info.append(rospy.Time.now())
    info.append(msg)
    rospy.loginfo(msg)
    event_info.append(info)
    save_events()
    save_poses()

atexit.register(save_data)

def gps_node(data):

    global count
    global flag
    global trigger

    position = []

    position.append(rospy.Time.now())
    position.append(data.latitude)
    position.append(data.longitude)
    position.append(data.altitude)
    position_info.append(position)

    if count == len(mission):
        count = 0

    geo_data = Geodesic.WGS84.Inverse(data.latitude, data.longitude, mission[count][0], mission[count][1])
    distance, bearing = geo_data['s12'], geo_data['azi2']

    if not flag:
        info = []
        msg = 'waypoint is {} m at {} degrees'.format(distance, bearing)
        rospy.loginfo(msg)
        info.append(rospy.Time.now())
        info.append(msg)
        event_info.append(info)

    if (distance <= error_tolerance and not flag) and trigger:

        trigger = False
        if not flag:

            info = []
            msg = 'sending broadcast trigger to payload'
            rospy.loginfo(msg)
            rospy.set_param('trigger/command', True)
            info.append(rospy.Time.now())
            info.append(msg)
            event_info.append(info)

            info= []
            msg = 'waiting for mission state'
            info.append(rospy.Time.now())
            rospy.loginfo(msg)
            info.append(msg)
            flag = True
            event_info.append(info)

    if rospy.get_param('trigger/acknowledgement'):

        info = []
        msg = 'acknowledgement received, adjusting flags'
        rospy.loginfo(msg)
        flag = False
        rospy.set_param('trigger/command', False)
        info.append(rospy.Time.now())
        info.append(msg)
        event_info.append(info)

        info = []
        msg = 'mission complete: continuing to next waypoint'
        count = count + 1
        rospy.loginfo(msg)
        info.append(rospy.Time.now())
        info.append(msg)
        event_info.append(info)

        rospy.set_param('trigger/acknowledgement', False)

def trigger_node(data):

    global cur_wp_index
    global trigger

    if cur_wp_index != data.wp_seq:

        cur_wp_index = data.wp_seq
        trigger      = True

        info = []
        msg  = 'waypoint at seq {} reached: {} {}'.format(cur_wp_index, mission[count][0], mission[count][1])
        rospy.loginfo(msg)
        info.append(rospy.Time.now())
        info.append(msg)
        event_info.append(info)
        info = []
        msg  = 'sending experiment_BEGIN trigger'
        rospy.loginfo(msg)
        info.append(rospy.Time.now())
        info.append(msg)
        event_info.append(info)


if __name__ == '__main__':

    try:

        rospy.init_node('ground_station', anonymous = True)
        # Uncomment for drone mission
        get_mission()
        # End uncomment
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_node)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, trigger_node)
        rospy.spin()

    except rospy.ROSInterruptException:

        pass
