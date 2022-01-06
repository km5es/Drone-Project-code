#/usr/bin/python3
'''
A simple Python3 script to receive GPSDO systematics and publish them on a SSD1306 128x64 OLED.
Uses Luma.OLED and ROS. Meant to work on a Raspberry Pi 4B.

author: Krishna Makhija
date: Jan 07th 2022
'''

import rospy
from std_msgs.msg import String
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306, ssd1325, ssd1331, sh1106
from time import sleep

serial = i2c(port=0, address=0x3C)
device = ssd1306(serial, rotate=0)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    # Box and text rendered in portrait mode
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, outline="white", fill="black")
        draw.text((10, 20), "I heard %s", data.data, fill="white")


def main():
    rospy.init_node('gpsdo_oled', anonymous=True)
    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()

