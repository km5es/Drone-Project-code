
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Quaternion
import numpy as np



def odom_cb(data):
    #rospy.loginfo(data.pose.pose.orientation)
    q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, 
                data.pose.pose.orientation.z, data.pose.pose.orientation.w]

    euler_angs = np.rad2deg(euler_from_quaternion(q, axes='sxyz'))
    #rospy.loginfo("Euler angles: %s, Quarternions: %s" %(euler_angs, q))
    rospy.loginfo("Roll: %s \n"
                    "Pitch: %s "%(euler_angs[0], euler_angs[1]))
    #rospy.loginfo(q)


def main():
    rospy.init_node('imu_data', anonymous=True)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_cb)
    rospy.spin()

if __name__ == '__main__':
    main()

'''

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = np.rad2deg(euler_from_quaternion (orientation_list))
    print roll, pitch, yaw

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/mavros/local_position/odom', Odometry, get_rotation)
rospy.spin()

#r = rospy.Rate(1)
#while not rospy.is_shutdown():
#    r.sleep()

'''