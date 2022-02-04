#!/usr/bin/python3
"""
Create ROS talker node to retrieve GPSDO systematics and publish them.
These are the systematics being published:
SYNC:FEE? Frequency error estimate
SYNC:LOCK? PLL lock state
SYNC:TINT? timing shift between 1PPS and GPS
MEAS:TEMP? measure temperature on GPSDO
GPS:SAT:VIS:COUN? number of GPS sats visible
PTIM:TIME:STR? current UTC time in string format

author: Krishna Makhija
date: 3rd Feb 2022
"""

import time, rospy
import pyvisa as visa
from pyvisa.constants import StopBits, Parity
from pyvisa import constants
from std_msgs.msg import Float32, String, ByteMultiArray, Float64MultiArray
from termcolor import colored


### Connect to GPSDO
#TODO: assign label to each unit like with the serial radios
rm = visa.ResourceManager()
print(rm.list_resources())

try:
    gpsdo_base = rm.open_resource('ASRL/dev/ttyUSB0::INSTR', baud_rate=115200, 
        data_bits=8, parity=Parity.none, stop_bits=StopBits.one, timeout=3000)
    gpsdo_base.set_visa_attribute(constants.VI_ATTR_ASRL_FLOW_CNTRL, constants.VI_ASRL_FLOW_NONE)
    gpsdo_base.write_termination = "\n"
    gpsdo_base.read_termination = "\n"
    ### Make sure the base is connected
    gpsdo_base.query('*IDN?')
    idn = gpsdo_base.read_bytes(50)
    if 'Fury' in idn.decode("utf-8"):
        print(colored("GPSDO base (Fury Firmware Rev 1.23) detected. Starting GPSDO ROS node now.", 'green'))
    else:
        print(colored("Unit detected: %s", idn, 'red'))
except:
    print("No GPSDO detected. Program will be terminated.")
        


class gpsdo_data:
    def __init__(self):
        self.data = []

    def idn(self):
        gpsdo_base.query('*IDN?')
        idn = gpsdo_base.read_bytes(40).decode("utf-8")
        return idn

    def fee(self):
        gpsdo_base.query('SYNC:FEE?')
        fee = gpsdo_base.read_bytes(16).decode("utf-8")
        return fee[:8]

    def lock(self):
        gpsdo_base.query('SYNC:LOCK?')
        lock = gpsdo_base.read_bytes(2).decode("utf-8")
        return lock[:1]

    def time(self):
        gpsdo_base.query('PTIM:TIME:STR?')
        time = gpsdo_base.read_bytes(16).decode("utf-8")
        return time[:8]

    def tint(self):
        gpsdo_base.query('SYNC:TINT?')
        tint = gpsdo_base.read_bytes(16).decode("utf-8")
        return tint[:8]


def talker():
    pub = rospy.Publisher('gpsdo_base/fee', String, queue_size=1)
    rospy.init_node('gpsdo_base_node', anonymous=True)
    rate = rospy.Rate(4) # 2 Hz
    data = gpsdo_data()
    while not rospy.is_shutdown():
        rospy.loginfo(data.fee() + '\t' + data.time())
        pub.publish(data.fee() + '\t' + data.time())
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
