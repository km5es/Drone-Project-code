#!/usr/bin/env python2.7
# client
"""
Base station code. Looks for when the drone has reached a waypoint and triggers payload to begin transmitting the
cal signal. There is also a concurrent thread that allows for manual initialization of the cal signal and/or
forced shutdown of the payload and base station.

Author: Krishna Makhija
data: 6th August 2020
"""
#TODO: should there be a heartbeat thread/process as well to ensure that serial comms are working?

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
#import psutil
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget

##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
client_script_name  = 'tcp_toggle.py'               # TCP client name
path                = '/home/kmakhija/'                           # path for saving SDR data
logs_path           = path + 'catkin_ws/src/Drone-Project-code/logs/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_base_station_events.log")
log_events          = open(log_name, "w") 
startup_initiate    = 'pay_INIT'                    # check to see if payload is running
startup_confirm     = 'INITconf'                    # confirmation msg from payload if running
handshake_start     = 'is_comms'                    # begin handshake prior to save data
handshake_conf      = 'serialOK'                    # confirmation from payload before save
toggle_ON           = 'start_tx'                    # message to payload to start cal
toggle_OFF          = 'stop_acq'                    # message from payload to stop saving
shutdown            = 'shutdown'                    # force shutdown of all SDRs
startup_SDR         = 'beginSDR'                    # boot up SDR codes.
reboot_payload      = '_reboot_'                    # reboot payload computer
acq_event           = Event()                       # save radio data
timeout             = 4                             # time after which saving data will stop if no trigger
repeat_keyword      = 4                             # number of times to repeat a telem msg
ser                 = serial.Serial('/dev/ttyPAYLOAD', 57600)
ser_timeout         = serial.Serial('/dev/ttyPAYLOAD', 57600, timeout=2)


client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
log_events.write("Timestamp\tSerial Data\tEvent\n")


if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")


### Define objects

def send_telem(keyword, serial_object, repeat_keyword):
    """
    Send keyword over telemetry radio for a total of repeat_keyword times.
    """
    for n in range(repeat_keyword):
        new_keyword = keyword + n*keyword
    serial_object.write(new_keyword)


def reset_buffer():
    """
    Clear telemetry radio buffers whenever possible.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def get_timestamp():
    """
    Returns current time for data logging.
    """
    timestring = time.strftime("%H%M%S-%d%m%Y")
    return timestring


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    while True:
        sleep(1e-6)
        if acq_event.is_set():
            print('Trigger from payload recd. Saving data now.')
            timestring      = get_timestamp()         
            filename        = path + timestring + str("_milton.dat")
            f               = open(filename, "w")
            print(colored('Saving data now in ' + str(filename), 'cyan'))
#                iocnt1 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
            start           = time.time()
            start_timeout   = start + timeout            
            while True:
                SDRdata     = client.recv(4096*8*16, socket.MSG_WAITALL)
                f.write(SDRdata)
                if acq_event.is_set() == False:
                    break               
                elif time.time() > start_timeout:
                    print(colored('No stop_acq message received from drone. Acquisition timed out in ' +str(timeout) + ' seconds.', 'grey', 'on_magenta'))
                    log_events.write(get_timestamp() + "\t\tNo stop_acq recd, TIMEOUT\n")
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
        log_events.write(get_timestamp() + "\t" + get_startup_confirmation + "\n")
        if startup_confirm in get_startup_confirmation:
            print(colored('Communication to the payload is UP. Waiting for trigger from drone.', 'green'))
            log_events.write("\t\tInitial comms with payload UP\n")
        else:
            print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
            log_events.write("\t\tInitial comms with payload DOWN\n")
    else:
        print(colored('No serial connection', 'magenta'))
        log_events.write(get_timestamp() + "\t\t" + "Serial comms on BASE DOWN\n")
    while not rospy.is_shutdown():
        sleep(1e-6)
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
            log_events.write(get_timestamp() + "\t\t" + "Drone reached WP.\n")
#            ser.write(handshake_start)
            send_telem(handshake_start, ser, repeat_keyword)
            get_handshake_conf = ser_timeout.read(msg_len*repeat_keyword)
            log_events.write(get_timestamp() + "\t" + get_handshake_conf + "\n")
            if handshake_conf in get_handshake_conf:
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                log_events.write("\t\tHandshake recd, toggle SDR ON\n")
#                ser.write(toggle_ON)
                send_telem(toggle_ON, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                print(get_stop_acq_trigger)
                log_events.write(get_timestamp() + "\t" + get_stop_acq_trigger + "\n")
                if toggle_OFF in get_stop_acq_trigger:
                    log_events.write("\t\tData acquisition toggled OFF\n")
                    acq_event.clear()
                    rospy.set_param('trigger/acknowledgement', True)
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                log_events.write(get_timestamp() + "\t\tHandshake failed no acquisition\n")
                rospy.set_param('trigger/acknowledgement', True)
                reset_buffer()
                pass


def manual_trigger_events():
    '''
    Manually trigger payload and initiate saving data on base station.
    '''
    while True:
        sleep(1e-6)                                     
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


def heartbeat():
    """
    Send telem heartbeat to ensure payload comms are okay.
    """


def main():
    try:
        t1 = Thread(target = recv_data)
        t2 = Thread(target = ros_events)
        t3 = Thread(target = manual_trigger_events)
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


if __name__ == '__main__':
    main()