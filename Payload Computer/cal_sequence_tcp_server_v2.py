#!/usr/bin/env python2.7
# server.py
'''
This script will combine with gr_cal_tcp_loopback_client.py to generate a cal signal from the drone. It reads from serial radio for
a trigger to begin the sequence. Once that is received, a TCP connection is established with the aforementioned GRC script, and a file
containing a predefined waveform is streamed over the TCP link. Once a certain number of pulses are reached, the RF switch is toggled
using GPIO. That number is defined as togglePoint and 2x togglePoint causes the sequence to finish.

Author: Krishna Makhija
date: 21st July 2020
v2.0 now will transmit zeros while the trigger is not set. This will ensure an integer number of cycles on the LO to complete before the
cal signal is transmitted. This ensures each transmission is phase consistent with the previous one.
'''
#FIXME: verify integrity of waveform
#TODO: identify serial radios using vendor and make. Important.
#TODO: redirect all print statements to a logfile. 
#TODO: save GPS and temp info in a log file. Also save IMU data.
#FIXME: Buffer underruns.

import socket
from termcolor import colored
import serial
import os
from datetime import datetime
import sys
from threading import Thread, Event

### Define global variables
port = 8810
togglePoint = 96                           ### number of pulses after which GPIO is toggled
ser = serial.Serial('/dev/ttyUSB0', 57600)  # timeout?
sample_packet = 4096*16                     #  Length of one pulse. might have to be changed to 16*4096 once the OFF time has been changed.
client_script_name = 'gr_cal_tcp_loopback_client.py'
s = socket.socket()                         # Create a socket object
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
host = socket.gethostbyname('127.0.0.1')    # Get local machine name
s.bind((host, port))                        # Bind to the port
s.listen(5)                                 # Now wait for client connection.
trigger_msg = 'start_tx'                    # change this to something more substantial later on
trigger_endacq = 'stop_acq'
shutdown = 'shutdown'
handshake_start = 'is_comms'
handshake_conf = 'serialOK'
trigger_event = Event()
stop_acq_event = Event()

### Make TCP and serial connections
print(colored('TCP server listening for connection from GRC flowgraph.', 'green'))
conn, addr = s.accept()
print(colored('Connection to GRC flowgraph established on ' + str(addr), 'green'))

if ser.isOpen() == True:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(colored('Serial connection to base is UP. Waiting for trigger.', 'green'))
    print(ser)

if len(trigger_msg) == len(trigger_endacq) == len(shutdown):
    msg_len = len(trigger_msg)

### Define objects
def reset_buffer():
    ser.reset_input_buffer()
    ser.reset_output_buffer()

def stream_file():
    '''
    Stream zeros unless a trigger is set. When triggered transmit cal sequence.
    '''
    zeros = open('zeros', 'rb')
    condition_LO = zeros.read(sample_packet)
    while (condition_LO):
        conn.send(condition_LO)
        if trigger_event.is_set():
            trigger_event.clear()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            filename='qpsk_waveform'
            print(colored('Trigger from base received at GPS time: ' +str(timestamp_start) + '. Beginning cal sequence using ' +str(filename), 'green'))
            pulses = 0
            for pulses in range(togglePoint):
                f = open(filename,'rb')
                cal_signal = f.read(sample_packet)
                while (cal_signal):
                    conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint/2:
                    print(colored("Switching polarization now.", 'cyan')) ### replace with GPIO command
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            stop_acq_event.set()
            print(colored('Calibration sequence complete at GPS time: ' +str(timestamp_stop) + '. Sending trigger to base and awaiting next trigger.', 'green'))

def serial_radio_events():
    '''
    This object is for serial comms. When a handshake request is received, an event will be set in the stream_file() object which 
    will begin the calibration.
    '''
    while True:
        get_handshake = ser.read(msg_len)
        if get_handshake == handshake_start:
            print(colored('Received handshake request from base station.', 'cyan'))
            ser.write(handshake_conf)
            reset_buffer()
            get_trigger_from_base = ser.read(msg_len)
            if get_trigger_from_base == str(trigger_msg):
                trigger_event.set()
                while True:
                    if stop_acq_event.is_set():
                        stop_acq_event.clear()
                        ser.write(trigger_endacq)
                        reset_buffer()
        elif get_handshake == str(shutdown):
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            print(colored('Kill command from base received. Shutting down TCP server and client programs.', 'red'))
            break

if __name__ == '__main__':
    t1 = Thread(target=serial_radio_events)
    t2 = Thread(target=stream_file)
    t1.start()
    t2.start()
    t1.join()
    t2.join()