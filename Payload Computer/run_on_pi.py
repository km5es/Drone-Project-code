#!/usr/bin/env python2.7
"""
Run vcgencmd to see if the Pi is overheating and/or throttling during missions.

author: Krishna Makhija
date: Jul 18th 2021
"""

import os
import time
from os.path import expanduser


path            = expanduser("~") + "/"             # data files save path
logs_path       = path + '/catkin_ws/src/Drone-Project-code/logs/metadata/'             
pi_cpu          = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_pi_cpu.log")



def measure_temp():
        temp = os.popen("vcgencmd measure_temp").readline()
        return (temp.replace("temp=",""))


def get_throttled():
        throttle = os.popen("vcgencmd get_throttled").readline()
        return (throttle.replace("throttled=",""))


def get_timestamp():
    """
    Returns current time for data logging.
    """
    time_now = time.time()
    mlsec = repr(time_now).split('.')[1][:3]
    time_now = time.strftime("%H:%M:%S.{}-%d/%m/%Y".format(mlsec))
    return time_now


def main():
        pi_cpu_a = open(pi_cpu, "a+")
        pi_cpu_a.write("Timestamp\tTemperature (deg C)\tThrottle register\n")
        #pi_cpu_w.close()
        #pi_cpu_a = open(pi_cpu, "a+")
        while True:
                time.sleep(5)
                current_time = get_timestamp()
                current_temp = measure_temp()
                current_thro = get_throttled()
                pi_cpu_a.write("%s\t%s\t%s\n" % (current_time, current_temp, current_thro))


if __name__ == '__main__':
    main()