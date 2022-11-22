#!/usr/bin/env python

"""
This code will load up WPs from a target .waypoints file. It will push the first WP to the FC. 
When a calibration sequence has completed it will update the WP table on the FC.
If the list of WPs has run out (IndexError), it will force an RTL.

It will also look for manual RC input to update WP table if the drone gets stuck at a WP.

author  : Krishna Makhija
last rev: 19th Nov 2022
"""
#TODO: add logging for all events. Create a separate log file for this node.
    #FIXME: does not work once ROS init_node has been called. what do? ROS logging sucks

import rospy, time, logging, imp
import csv
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
from os.path import expanduser
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *


filename    = "mission.waypoints"
path        = expanduser("~")  + "/catkin_ws/src/Drone-Project-code/mission/"
filename    = path + filename
rospy.set_param('trigger/waypoint', False)
rospy.set_param('trigger/sequence', False)
rospy.set_param('trigger/lock', False)          # block wp_trigger at each WP
timeout         = 12                            # timeout before updating new WP
lock_timeout    = 5                             # time to unlock trigger/sequence
pwm_threshold   = 1600                          # pulse width (us) above which trigger is set from RC

#* log all events for troubleshooting
home_path           = expanduser("~") + "/"         # define home path
logs_path           = home_path + '/catkin_ws/src/Drone-Project-code/logs/payload/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_write_WPs.log")
logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)


class pwm_reader:
    """
    A class to read PWM pulses and calculate their frequency
    and duty cycle.  The frequency is how often the pulse
    happens per second.  The duty cycle is the percentage of
    pulse high time per cycle.
    """
    def __init__(self, pi, gpio, weighting=0.0):
        """
        Instantiate with the Pi and gpio of the PWM signal
        to monitor.
        
        Optionally a weighting may be specified.  This is a number
        between 0 and 1 and indicates how much the old reading
        affects the new reading.  It defaults to 0 which means
        the old reading has no effect.  This may be used to
        smooth the data.
        """
        self.pi = pi
        self.gpio = gpio
        if weighting < 0.0:
            weighting = 0.0
        elif weighting > 0.99:
            weighting = 0.99
        self._new = 1.0 - weighting # Weighting for new reading.
        self._old = weighting       # Weighting for old reading.
        self._high_tick = None
        self._period = None
        self._high = None
        pi.set_mode(gpio, pigpio.INPUT)
        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        if level == 1:
            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)
                if self._period is not None:
                    self._period = (self._old * self._period) + (self._new * t)
                else:
                    self._period = t
            self._high_tick = tick
        elif level == 0:
            if self._high_tick is not None:
                t = pigpio.tickDiff(self._high_tick, tick)
                if self._high is not None:
                    self._high = (self._old * self._high) + (self._new * t)
                else:
                    self._high = t

    def frequency(self):
        """
        Returns the PWM frequency.
        """
        if self._period is not None:
            return 1000000.0 / self._period
        else:
            return 0.0

    def pulse_width(self):
        """
        Returns the PWM pulse width in microseconds.
        """
        if self._high is not None:
            return self._high
        else:
            return 0.0

    def duty_cycle(self):
        """
        Returns the PWM duty cycle percentage.
        """
        if self._high is not None:
          return 100.0 * self._high / self._period
        else:
            return 0.0

    def cancel(self):
        """
        Cancels the reader and releases resources.
        """
        self._cb.cancel()


def waypoint_clear_client():
    """
    Clear WPs from the FC.
    """
    try:
        response = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
        return response.call().success
    except rospy.ServiceException, e:
        print "Service call for mission clear failed: %s" % e
        logging.debug('Service call for mission clear failed.')
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
            logging.info("WP table updated %s" %wl)
        else:
            print("Write mission error.")
            logging.debug("Write mission error.")
    except rospy.ServiceException, e:
        print("Service call for WP push failed: %s" %e)
        logging.debug("Service call for WP push failed %s" %e)


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
        logging.debug("Set current WP failed %s" %e)


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
    Those conditions are when drone has reached WP and is stable
    and/or when there is manual input from the RC (switch SH)
    """
    global wl

    #* set up PWM monitoring variables
    PWM_GPIO    = 18
    pi          = pigpio.pi()
    p           = pwm_reader(pi, PWM_GPIO)

    #* set up WP table parameters and push first WP
    n = 2
    rospy.init_node('write_WP', anonymous = True)
    imp.reload(logging)     #? reload because ROS breaks other methods of logging
    logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)
    logging.info("ROS node write_WPs.py has been initiated.")
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
        time.sleep(0.1)

        #* wait for ROS flag from autonomy pipeline and update WP table 
        #* clear waypoint table before ending sequence so that it does not re-trigger.
        if rospy.get_param('trigger/sequence') == True:
            waypoint_clear_client()
        if rospy.get_param('trigger/waypoint') == True:
            rospy.set_param('trigger/waypoint', False)
            try:
                print("trigger/waypoint flag from cal_seq recd. Updating WP table.")
                logging.info("trigger/waypoint flag from cal_seq recd. Updating WP table.")
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
                logging.info("trigger/waypoint flag from cal_seq recd. Doing an RTL now.")
                try:
                    #FIXME: the coords here should be different because this is triggering the sequence
                    wp = create_waypoint(20, 0, float(reader[n][8]), float(reader[n][9]), float(reader[n][10]))
                    wl.append(wp)
                    push_wp()
                    change_mode()
                except IndexError:
                    pass

        #* read RC input and update WP table
        pulse_width = p.pulse_width()
        if pulse_width > pwm_threshold:
            print('Manual RC input (pw=%s) for calibration recd.' %pulse_width)
            logging.info('Manual RC input (pw=%s) for calibration recd.' %pulse_width)
            rospy.set_param('trigger/sequence', True)
            time.sleep(3.5)     # avoid multiple pushes to FC


if __name__ == '__main__':
    main()

