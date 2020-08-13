#!/usr/bin/env python2.7
# server
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
#TODO: integrate the circular polarized waveform as well.
#TODO: identify serial radios using vendor and make. Important.
#TODO: redirect all print statements to a logfile. 
#TODO: save GPS and temp info in a log file. Also save IMU data.

import socket
from termcolor import colored
import serial
import os
from datetime import datetime
import sys
from threading import Thread, Event
import time


### Define global variables

port                = 8810
togglePoint         = 96                                    # number of pulses after which GPIO is toggled
sample_packet       = 4096*16                               # Length of one pulse.
ser                 = serial.Serial('/dev/ttyUSB0', 57600)  
ser_timeout         = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)
s                   = socket.socket()                       # Create a socket object
host                = socket.gethostbyname('127.0.0.1')     # Get local machine name
startup_initiate    = 'pay_INIT'                            # check to see if payload is running
startup_confirm     = 'INITconf'                            # confirmation msg from payload if running
handshake_start     = 'is_comms'                            # begin handshake prior to save data
handshake_conf      = 'serialOK'                            # confirmation from payload before save
toggle_ON           = 'start_tx'                            # message to payload to start cal                
toggle_OFF          = 'stop_acq'                            # message from payload to stop saving
shutdown            = 'shutdown'                            # force shutdown of all SDRs
repeat_keyword      = 4
client_script_name  = 'gr_cal_tcp_loopback_client.py'
trigger_event       = Event()
stop_acq_event      = Event()


### Make TCP and serial connections

os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))                                        # Bind to the port
s.listen(5)                                                 # Now wait for client connection.
conn, addr = s.accept()
print(colored('TCP server listening for connection from GRC flowgraph.', 'green'))
print(colored('Connection to GRC flowgraph established on ' + str(addr), 'green'))

if ser.isOpen() == True:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(colored('Serial connection to payload is UP. Waiting for trigger.', 'green'))
    print(ser)

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


def reset_buffer():
    """
    Clear telemetry radio buffers whenever possible.
    """
    ser.reset_input_buffer()
    ser.reset_output_buffer()


def stream_file():
    '''
    Stream zeros unless a trigger is set. When triggered transmit cal sequence.
    '''
    zeros = open('zeros', 'rb')
    condition_LO = zeros.read()
    filename = 'qpsk_waveform'
    f = open(filename,'rb')
    cal_signal = f.read()
    while trigger_event.is_set() == False:
        conn.send(condition_LO)
        if trigger_event.is_set():
            start = time.time()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            print(colored('Trigger from base received at GPS time: ' +str(timestamp_start) + '. Beginning cal sequence using ' +str(filename), 'green'))
            pulses = 0
            for pulses in range(togglePoint):
                conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint/2:
                    print(colored("Switching polarization now.", 'cyan')) ### replace with GPIO command
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            stop_acq_event.set()
            print(colored('Calibration sequence complete at GPS time: ' +str(timestamp_stop) + '. Total time taken was: ' + str(total_time) + ' seconds. Sending trigger to base and awaiting next trigger.', 'green'))
            trigger_event.clear()


def serial_radio_events():
    '''
    This object is for serial comms. When a handshake request is received, an event will be set in the stream_file() object which 
    will begin the calibration. When calibration is finished an event is set in stream_file() which sends a serial msg from this
    object to the base to stop acquisiiton.
    '''
    while True:
        get_handshake = ser.read(msg_len*repeat_keyword)
#        if get_handshake == handshake_start:
        if handshake_start in get_handshake:
            print(colored('Received handshake request from base station.', 'cyan'))
#            ser.write(handshake_conf)
            send_telem(handshake_conf, ser, repeat_keyword)
            reset_buffer()
            get_trigger_from_base = ser_timeout.read(msg_len*repeat_keyword) ### set timeout here for handshake   
#            if get_trigger_from_base == str(toggle_ON):
            if toggle_ON in get_trigger_from_base:
                trigger_event.set()
                while trigger_event.is_set() == True:
                    if stop_acq_event.is_set():
                        stop_acq_event.clear()
                        time.sleep(0.25)                                    ### buffer time for the receiver to "catch up".
#                        ser.write(toggle_OFF)
                        send_telem(toggle_OFF, ser, repeat_keyword)
                        reset_buffer()
            else:
                print(colored('No start cal trigger recd from base. Waiting for next handshake request', 'magenta'))
                pass
#        elif get_handshake == startup_initiate:
        elif startup_initiate in get_handshake:
            print(colored('The base has started up and is talking.', 'grey', 'on_green'))
#            ser.write(startup_confirm)
            send_telem(startup_confirm, ser, repeat_keyword)
            reset_buffer()
#        elif get_handshake == str(shutdown):
        elif shutdown in get_handshake:
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