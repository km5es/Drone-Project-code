#!/usr/bin/env python2.7
# client
"""
Base station code. Looks for when the drone has reached a waypoint and triggers payload to begin transmitting the
cal signal. There is also a concurrent thread that allows for manual initialization of the cal signal and/or
forced shutdown of the payload and base station.

Author: Krishna Makhija
data: 6th August 2020
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
path                = '/home/kmakhija/'             # data files save path
startup_initiate    = 'pay_INIT'                    # check to see if payload is running
startup_confirm     = 'INITconf'                    # confirmation msg from payload if running
handshake_start     = 'is_comms'                    # begin handshake prior to save data
handshake_conf      = 'serialOK'                    # confirmation from payload before save
toggle_ON           = 'start_tx'                    # message to payload to start cal
toggle_OFF          = 'stop_acq'                    # message from payload to stop saving
shutdown            = 'shutdown'                    # force shutdown of all SDRs
acq_event           = Event()
timeout             = 4                             # time after which saving data will stop if no trigger
repeat_keyword      = 4                             # number of times to repeat a telem msg
ser                 = serial.Serial('/dev/ttyUSB0', 57600)
ser_timeout         = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)

client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))

if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)

### Define objects

def send_telem(keyword, serial_object, repeat_keyword):
    """
    Send keyword over telemetry radio for a total of repeat_keyword times.
    """
    for n in range(repeat_keyword):
        new_keyword = keyword + n*keyword
    serial_object.write(new_keyword)


def recv_telem(recvd_msg, keyword):
    """
    Search received telem message for operative keyword. Return True is keyword is found. False otherwise.
    """
    return keyword in recvd_msg


def reset_buffer():
    """
    Clear telemetry radio buffers whenever possible.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def temp_connect():
    client_temp  = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    ip_temp      = socket.gethostbyname("127.0.0.1")
    port_temp    = 7890
    address      = (ip_temp, port_temp)
    client_temp.connect((address))
#    temp_data    = client_temp.recv(32)


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    while True:
        if acq_event.is_set():
            print('Trigger from payload recd. Saving data now.')
            timestring      = time.strftime("%H%M%S-%d%m%Y")         
            filename        = path + timestring + str("_milton.dat")
            f               = open(filename, "w")
            print(colored('Saving data now in ' + str(filename), 'cyan'))
#                iocnt1 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
#            file_meta       = path + timestring + str("_meta.dat")
#            f_meta          = open(file_meta, "w")
#            temp_connect()
            start           = time.time()
            start_timeout   = start + timeout            
            while True:
                SDRdata     = client.recv(4096*8*16, socket.MSG_WAITALL)
                f.write(SDRdata)
#                temp_data   = client_temp.recv(32)
#                f_meta.write(temp_data)
                if acq_event.is_set() == False:
                    break               
                elif time.time() > start_timeout:
                    print(colored('No stop_acq message received from drone. Acquisition timed out in ' +str(timeout) + ' seconds.', 'grey', 'on_magenta'))
                    acq_event.clear()
                    rospy.set_param('trigger/acknowledgement', True)
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
        print(ser, ser_timeout)
        reset_buffer()
#        ser.write(startup_initiate)
        send_telem(startup_initiate, ser, repeat_keyword)
        get_startup_confirmation = ser_timeout.read(msg_len*repeat_keyword)
        if startup_confirm in get_startup_confirmation:
            print(colored('Communication to the payload is UP. Waiting for trigger from drone.', 'green'))
        else:
            print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
    else:
        print(colored('No serial connection', 'magenta'))
    while not rospy.is_shutdown():
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
#            ser.write(handshake_start)
            send_telem(handshake_start, ser, repeat_keyword)
            get_handshake_conf = ser_timeout.read(msg_len*repeat_keyword)
            if handshake_conf in get_handshake_conf:
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
#                ser.write(toggle_ON)
                send_telem(toggle_ON, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                print(get_stop_acq_trigger)
                if toggle_OFF in get_stop_acq_trigger:
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
#        ser.write(msg)
        send_telem(msg, ser, repeat_keyword)
        if msg == str(shutdown):
            print(colored('Shutting down payload and this code.', 'red'))
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
            pass
        elif msg == str(handshake_start):
            get_handshake_conf = ser_timeout.read(msg_len)
            print(get_handshake_conf)
            if handshake_conf in get_handshake_conf:
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
#                ser.write(toggle_ON)
                send_telem(toggle_ON, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                print(get_stop_acq_trigger)
                if toggle_OFF in get_stop_acq_trigger:
                    acq_event.clear()
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                pass


if __name__ == '__main__':
    try:
        t1 = Thread(target=recv_data)
        t2 = Thread(target=ros_events)
#        t3 = Thread(target=manual_trigger_events)
        t1.start()
        t2.start()
#        t3.start()
        t1.join()
        t2.join()
#        t3.join()
    except (serial.SerialException, socket.error):
        print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
