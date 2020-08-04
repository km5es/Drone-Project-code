#!/usr/bin/env python2.7
"""
Author: KM
Test sync between SDRs only. Do not include ROS.
"""
#TODO: Test data rate calculator.
#TODO: advance this to a new version of open-loop-accept-tcp.py
#TODO: keep the raw_input() call so that condition 3 can be calibrated for.

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

##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
client_script_name  = 'tcp_toggle.py'
path                = '/home/kmakhija/'
toggle_ON           = 'start_tx'
toggle_OFF          = 'stop_acq'
handshake_start     = 'is_comms'
handshake_conf      = 'serialOK'
shutdown            = 'shutdown'
acq_event           = Event()
timeout             = 4
ser                 = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)


client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))


def reset_buffer():
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    print('Waiting for trigger from payload to begin saving data.')
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
                    print(colored('No stop_acq message received from drone. Acquisition timed out in ' +str(timeout) + ' seconds.', 'magenta'))
                    acq_event.clear()
                    break
            end = time.time()
#                iocnt2 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
            print(colored('\nFinished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'green'))


def serial_radio_events():
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
            get_handshake_conf = ser.read(len(toggle_ON))
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
                print(colored('Handshake with drone comms failed. No data will be saved.', 'magenta'))
                pass


if __name__ == '__main__':
    try:
        t1 = Thread(target=recv_data)
        t2 = Thread(target=serial_radio_events)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
    except (serial.SerialException, socket.error):
        print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
