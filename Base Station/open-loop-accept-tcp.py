#%%
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

##### Define global variables
client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip=socket.gethostbyname("127.0.0.1")
port=8800
address=(ip,port)
client.connect((address))  ## <--Add this line.

toggle_ON = 'start_tx'
toggle_OFF = 'stop_acq'
nullSink = open(os.devnull, 'w')
samp_rate = 50e6
acquire_time = 3
data_len = int(acquire_time*samp_rate/4096)      ### total number of samples to be acquired. 8 bytes per sample in float32 per channel.
event_end = Event()

print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
ser = serial.Serial('/dev/ttyUSB1', 57600) ### which serial radio is doing what? this is drone

def saveData():
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    while not rospy.is_shutdown():
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Received trigger from drone. Triggering payload.', 'cyan'))
            ser.write(toggle_ON) ### tell ODROID to transmit
            timestring = time.strftime("%H%M%S-%d%m%Y")         ###filename is timestamp. location is path to this script.
            filename = timestring + str("_milton.dat")
            f = open(filename, "w")
            print(colored('Saving data now in ' + str(filename), 'cyan'))
            start = time.time()
#            while i < data_len:                     ### clear TCP buffer before saving. it might be full. alternatively just save into one array and then dump.
            while True:
                SDRdata = client.recv(8*4096, socket.MSG_WAITALL)    ### 8 bytes for each sample consisting of float32. change to 4 when switching to int. 
                f.write(SDRdata)            ### might need to switch to socket.recv_into(buffer[, nbytes[, flags]]). experiment and see.
                if event_end.is_set():
                    event_end.clear()   # reset event flag for next WP
                    break
            end = time.time()
            rospy.set_param('trigger/acknowledgement', True)
            print(colored('Finished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'green'))


def stop_acq():
    while True: 
        a = ser.read(8)
        if a == str(toggle_OFF):
            event_end.set()
            print("Setting event now.")

if __name__ == '__main__':
    while True:
        try:
            t1 = Thread(target=saveData)
            t2 = Thread(target=stop_acq)
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        except (serial.SerialException):
            print(colored("USB serial device busy. Killing process using USB0 and trying again.", 'red'))
            os.system('kill -9 $(fuser /dev/ttyUSB0)')
