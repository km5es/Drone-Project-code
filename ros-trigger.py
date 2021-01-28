#!/usr/bin/env python2
"""
Rewrite Varundev's ROS code so that it does not look for a csv file instead just sets a trigger when 
a waypoint is reached.

Author: KM
Date: 07/01/2021
modified: 27/01/2021
"""

import rospy, time
from os.path import expanduser
from termcolor import colored
from std_msgs.msg import Float32
from threading import Event, Thread
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import WaypointReached
from sensor_msgs.msg import Imu


rospy.set_param('trigger/command', False)
rospy.set_param('trigger/acknowledgement', False)
n = 0

def trigger_node(data):
    global n
    n = n + 1
    if data:
        print("Waypoint sequence #" +str(data.wp_seq) +" reached. Triggering GNUradio code.")
        rospy.set_param('trigger/command', True)


def imu_data(data):
    print(data)


def main():
    try:
        rospy.init_node('ground_station', anonymous = True)
        rospy.Subscriber('/mavros/mission/reached', WaypointReached, trigger_node)
#        rospy.Subscriber('/mavros/imu/data', Imu, imu_data)
        rospy.spin()
    except rospy.ROSInterruptException:

        pass


if __name__ == '__main__':
    main()
