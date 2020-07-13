#!/usr/bin/env python2.7
# server.py
'''
This script will combine with gr_cal_tcp_loopback_client.py to generate a cal signal from the drone. It reads from serial radio for
a trigger to begin the sequence. Once that is received, a TCP connection is established with the aforementioned GRC script, and a file
containing a predefined waveform is streamed over the TCP link. Once a certain number of pulses are reached, the RF switch is toggled
using GPIO. That number is defined as togglePoint and 2x togglePoint causes the sequence to finish.

Author: Krishna Makhija
date: 21st Mar 2020, modified 26/3/2020
'''
#TODO: check with actual SDR. fix the underflow issue. might have to increase the size of each buffer
    #FIXME: verify integrity of waveform
#TODO: identify serial radios using vendor and make. Important.
#TODO: redirect all print statements to a logfile. 

import socket                   # Import socket module
from termcolor import colored
import serial
import os
from datetime import datetime
import sys

port = 8810
togglePoint = 100    ### number of pulses after which GPIO is toggled
ser = serial.Serial('/dev/ttyUSB0', 57600)  # timeout?
sample_packet = 4096*17     #  Length of one pulse. might have to be changed to 16*4096 once the OFF time has been changed.
client_script_name = 'gr_cal_tcp_loopback_client.py'

def reset_buffer():
    ser.reset_input_buffer()
    ser.reset_output_buffer()

def streamFile(togglePoint):
    global port                    # Reserve a port for your service.
    s = socket.socket()             # Create a socket object
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host = socket.gethostbyname('127.0.0.1')     # Get local machine name
    s.bind((host, port))            # Bind to the port
    s.listen(5)                     # Now wait for client connection.
    trigger_msg = 'start_tx'               # change this to something more substantial later on
    trigger_endacq = 'stop_acq'
    shutdown = 'shutdown'
    handshake_start = 'is_comms'
    handshake_conf = 'serialOK'
    if len(trigger_msg) == len(trigger_endacq) == len(shutdown):
        msg_len = len(trigger_msg)

#    sys.stdout = open("logfile.txt", "w")
    print(colored('TCP server listening for connection from GRC flowgraph.', 'green'))
    conn, addr = s.accept()     # Establish connection with client.
    print(colored('Connection to GRC flowgraph established on ' + str(addr), 'green'))
#    sys.stdout.close()
    if ser.isOpen() == True:
        reset_buffer()
        print(colored('Serial connection to base is UP. Waiting for trigger.', 'green'))
        print(ser)
    while True:
        reset_buffer()
        get_handshake = ser.read(msg_len)
        if get_handshake == handshake_start:
            ser.write(handshake_conf)
            get_trigger_from_base = ser.read(msg_len)
            print(get_trigger_from_base)
            if get_trigger_from_base == str(trigger_msg):
#                timestamp_start = time.strftime("%H%M%S-%d%m%Y")
                timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
                filename='qpsk_waveform'
                print(colored('Trigger from base received at GPS time: ' +str(timestamp_start) + '. Beginning cal sequence using ' +str(filename), 'green'))
                pulses = 0
                for pulses in range(togglePoint):
                    f = open(filename,'rb')
                    l = f.read(sample_packet)
                    while (l):
                        conn.send(l)
                        l = f.read(sample_packet)
                    pulses += 1
                    if pulses == togglePoint/2:
                        print(colored("Switching polarization now.", 'cyan')) ### replace with GPIO command
#                    f.close()
#                timestamp_stop = time.strftime("%H%M%S-%d%m%Y")
                timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
                print(colored('Calibration sequence complete at GPS time: ' +str(timestamp_stop) + '. Sending trigger to base and awaiting next trigger.', 'green'))
                ser.write(trigger_endacq)
#                conn.close()
            elif get_trigger_from_base == str(shutdown):
                os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
                print(colored('Kill command from base received. Shutting down TCP server and client programs.', 'red'))
                break

if __name__ == '__main__':
    os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
    streamFile(togglePoint)
#    except socket.error:
#        os.system('lsof -t -i tcp:8810 | xargs kill -9')
