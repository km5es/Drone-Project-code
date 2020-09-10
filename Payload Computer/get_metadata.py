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

path        = '/home/kmakhija/'             # data files save path
metadata    = path + 'meta.dat'


def main():
    """
    Initiate metadata file and write to it.
    """
    rospy.init_node('get_metadata', anonymous=True)
    rospy.set_param('trigger/metadata_ON', False)
    file = open(metadata, "w+")
    file.write("Timestamp\tLocal Position (x)\tLocal Position (y)\tLocal Position (z)\tSetpoint (x)\tSetpoint (y)\tSetpoint (z)\tTemperature\n")
    file.close()
    wp_num = 0
    print(colored('ROS metadata node intialized. Waiting for flag from SDR code to begin saving metadata.', 'green'))
    while not rospy.is_shutdown():
        time.sleep(0.001)
        if rospy.get_param('trigger/metadata_ON') == True:
            print(colored('Saving Waypoint #' + str(wp_num) + ' metadata in ' + str(metadata), 'grey', 'on_white'))
            file = open(metadata, "a+")
            file.write("Waypoint #%s\n" %(wp_num))
            wp_num += 1
            while True:
                current_time    = time.strftime("%H%M%S-%d%m%Y")
                local_pose      = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped)
                set_target      = rospy.wait_for_message('/mavros/setpoint_raw/target_local', PositionTarget)
#                sdr_temp        = rospy.wait_for_message('sdr_temperature', Float32)
                file.write("%s\t%s\t%s\t%s\t" %(current_time, local_pose.pose.position.x, local_pose.pose.position.y, local_pose.pose.position.z))
                file.write("%s\t%s\t%s\n" %(set_target.position.x, set_target.position.y, set_target.position.z))
                if rospy.get_param('trigger/metadata_OFF') == True:
                    rospy.set_param('trigger/metadata_ON', False)
                    print('Finished saving metadata for this WP.')
                    file.close()
                    break


if __name__ == '__main__':
    main()