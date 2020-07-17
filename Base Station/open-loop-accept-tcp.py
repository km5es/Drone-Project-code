#!/usr/bin/env python2.7
"""
Author: KM
Python code for receiving local loopback TCP data from GRC flowgraph entitled
tcp_toggle.grc. This is to begin saving data from a start point received from 
serial comms and save for a fixed length.
"""

#TODO: Would it be nice to have progress bars/size readouts of the files?
#FIXME: should the TCP connection be interleaved or not? The data rates are going to be fast as hell.
#FIXME: I suppose for now the rates dont need to be that high. Hmph.
#FIXME: why does it take so long to finish a loop? saving is taking longer than the sample rates. How slow is Python?
#TODO: Maybe implement a feature where the data rates are displayed in the std out?
#TODO: add a path for saving data to 
#TODO: add a feature wherein data save rates are displayed in MB/s 
#FIXME: The ROS code still looks at the mission.csv file for triggering? Confirm this also.
#FIXME: should there be some hand-shaking with the drone serial prior to each sequence? maybe there should be an exception if the other serial is not connected?
    #FIXME: break the else loop correctly.
#FIXME: Why isn't there an exception for socket error 98? Add sudo ls-f -t -i tcp:8800 | xargs kill -9

import socket
import serial
import os
import sys
from time import sleep
import rospy
from termcolor import colored
import timeit
import time
from serial.serialutil import SerialException
from threading import Thread, Event
import psutil

##### Define global variables
client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip=socket.gethostbyname("127.0.0.1")
port=8800
address=(ip,port)
client.connect((address)) 

toggle_ON = 'start_tx'
toggle_OFF = 'stop_acq'
handshake_start = 'is_comms'
handshake_conf = 'serialOK'
handshake_event = Event()
acq_event = Event()
timeout = 8
'''
nullSink = open(os.devnull, 'w')
samp_rate = 15.36e6
acquire_time = 3
data_len = int(acquire_time*samp_rate/4096)      ### total number of samples to be acquired. 8 bytes per sample in float32 per channel.
'''
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
ser = serial.Serial('/dev/ttyUSB0', 57600) ### which serial radio is doing what? this is drone

def reset_buffer():
    ser.reset_input_buffer()
    ser.reset_output_buffer()

def saveData():
    if ser.isOpen() == True:
        reset_buffer()
        print(colored('Serial connection to base is UP. Waiting for trigger.', 'green'))
        print(ser)
    else:
        print(colored('No serial connection', 'magenta'))
    while not rospy.is_shutdown():
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
            ser.write(handshake_start)
            if handshake_event.wait(timeout=1):
                reset_buffer()
                handshake_event.clear()
                print(colored('Received handshake from drone. Triggering calibration signal.', 'cyan'))
                ser.write(toggle_ON) ### tell payload to transmit
                timestring = time.strftime("%H%M%S-%d%m%Y")         ###filename is timestamp. location is path to this script.
                filename = timestring + str("_milton.dat")
                f = open(filename, "w")
                print(colored('Saving data now in ' + str(filename), 'cyan'))
#                iocnt1 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
                start = time.time()
                start_timeout = start + timeout
#                while i < data_len:                     ### clear TCP buffer before saving. it might be full. alternatively just save into one array and then dump.
                while True:
                    SDRdata = client.recv(8*4096, socket.MSG_WAITALL)    ### 8 bytes for each sample consisting of float32. change to 4 when switching to int. 
                    f.write(SDRdata)            ### might need to switch to socket.recv_into(buffer[, nbytes[, flags]]). experiment and see.
                    if acq_event.is_set():
                        acq_event.clear()   # reset acq_event flag for next WP
                        break
                    elif time.time() > start_timeout:
                        print(colored('No stop_acq message received from drone. Acquisition timed out in ' +str(timeout) + ' seconds.', 'magenta'))
                        break
                end = time.time()
#                iocnt2 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
                rospy.set_param('trigger/acknowledgement', True)
                reset_buffer()
                print(colored('Finished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'green'))
#                print('Blocks written {0}'.format(iocnt2.write_count - iocnt1.write_count))
#                print('Blocks read {0}'.format(iocnt2.read_count - iocnt1.read_count))
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'magenta'))

def stop_acq():
    while True: 
        a = ser.read(len(toggle_ON))
        if a == str(toggle_OFF):
            acq_event.set()
            print("Setting acq_event now")
        elif a == str(handshake_conf):
            handshake_event.set()
            print('Setting handshake_event now.')

if __name__ == '__main__':
    while True:
        try:
            t1 = Thread(target=saveData)
            t2 = Thread(target=stop_acq)
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        except (serial.SerialException, socket.error):
            print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
            os.system('kill -9 $(fuser /dev/ttyUSB0)')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
