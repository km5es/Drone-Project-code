#!/usr/bin/env python2.7
"""
Author: KM
Test sync between SDRs only. Do not include ROS.
"""
#TODO: Test data rate calculator.
#TODO: maybe instead of a short keyword there should be a sequence of the same keywords?
#TODO: metadata!

import socket
import serial
import os
import sys
from time import sleep
from termcolor import colored
import timeit
import time
from serial.serialutil import SerialException
from threading import Thread, Event
import psutil
import rospy

##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
client_script_name  = 'tcp_toggle.py'
path                = '/home/kmakhija/'
startup_initiate    = 'pay_INIT'
startup_confirm     = 'INITconf'
handshake_start     = 'is_comms'
handshake_conf      = 'serialOK'
toggle_ON           = 'start_tx'
toggle_OFF          = 'stop_acq'
shutdown            = 'shutdown'
acq_event           = Event()
timeout             = 4
ser                 = serial.Serial('/dev/ttyUSB0', 57600)
ser_timeout         = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)

client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))


def reset_buffer():
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    while True:
        if acq_event.is_set():
            print('Trigger from payload recd. Saving data now.')
            timestring = time.strftime("%H%M%S-%d%m%Y")         
            filename = path + timestring + str("_milton.dat")
            f = open(filename, "w")
            print(colored('Saving data now in ' + str(filename), 'cyan'))
#                iocnt1 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
            start = time.time()
            start_timeout = start + timeout
            while True:
                SDRdata = client.recv(4096*8*16, socket.MSG_WAITALL)
                f.write(SDRdata)
                if acq_event.is_set() == False:
                    break               
                elif time.time() > start_timeout:
                    print(colored('No stop_acq message received from drone. Acquisition timed out in ' +str(timeout) + ' seconds.', 'grey', 'on_magenta'))
                    rospy.set_param('trigger/acknowledgement', True)
                    acq_event.clear()
                    reset_buffer()
                    break
            end = time.time()
#                iocnt2 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
            print(colored('\nFinished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'grey', 'on_green'))


def ros_events():
    """
    Check for ROS-based events like when the drone has reached WP, and trigger data acquisition.
    """
    if ser.isOpen() == True:
        reset_buffer()
        ser.write(startup_initiate)
        get_startup_confirmation = ser_timeout.read(len(startup_confirm))
        if get_startup_confirmation == startup_confirm:
            print(colored('Communication to the payload is UP. Waiting for trigger from drone.', 'green'))
        else:
            print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
        print(ser, ser_timeout)
    else:
        print(colored('No serial connection', 'magenta'))
    while not rospy.is_shutdown():
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
            ser.write(handshake_start)
            get_handshake_conf = ser_timeout.read(len(handshake_conf))
            if get_handshake_conf == str(handshake_conf):
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                ser.write(toggle_ON)
                acq_event.set()
                get_stop_acq_trigger = ser.read(len(toggle_OFF))
                print(get_stop_acq_trigger)
                if get_stop_acq_trigger == str(toggle_OFF):
                    acq_event.clear()
                    rospy.set_param('trigger/acknowledgement', True)
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                rospy.set_param('trigger/acknowledgement', True)
                reset_buffer()
                pass


def manual_trigger_events():
    '''
    Manually trigger payload and initiate saving data on base station.
    '''
    while True:                                     
        msg = raw_input("Enter serial comms message here: ")        # send is_comms handshake request
        ser.write(msg)
        if msg == str(shutdown):
            print(colored('Shutting down payload and this code.', 'red'))
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
            pass
        elif msg == str(handshake_start):
            get_handshake_conf = ser_timeout.read(len(toggle_ON))
            print(get_handshake_conf)
            if get_handshake_conf == str(handshake_conf):
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                ser.write(toggle_ON)
                acq_event.set()
                get_stop_acq_trigger = ser.read(len(toggle_OFF))
                print(get_stop_acq_trigger)
                if get_stop_acq_trigger == str(toggle_OFF):
                    acq_event.clear()
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                pass


if __name__ == '__main__':
    try:
        t1 = Thread(target=recv_data)
        t2 = Thread(target=ros_events)
        t3 = Thread(target=manual_trigger_events)
        t1.start()
        t2.start()
        t3.start()
        t1.join()
        t2.join()
        t3.join()
    except (serial.SerialException, socket.error):
        print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
