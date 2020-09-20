#!/usr/bin/env python2.7
# client
"""
Base station code. Looks for when the drone has reached a waypoint and triggers payload to begin transmitting the
cal signal. There is also a concurrent thread that allows for manual initialization of the cal signal and/or
forced shutdown of the payload and base station. Logs are saved in the /logs directory of the repo.

Author: Krishna Makhija
data: 6th August 2020
"""
#TODO: should there be a heartbeat thread/process as well to ensure that serial comms are working?
    #FIXME: Heartbeat feature is not working properly. Timing it right seems tricky. 

import socket, serial, os, sys, rospy, logging, timeit, time
#import psutil
from time import sleep
from termcolor import colored
from os.path import expanduser
from serial.serialutil import SerialException
from threading import Thread, Event

##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
client_script_name  = 'tcp_toggle.py'               # TCP client name
path                = expanduser("~") + "/"         # define home path
logs_path           = path + 'catkin_ws/src/Drone-Project-code/logs/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_base_station_events.log") 
heartbeat_check     = 'hrt_beat'                    # heartbeat every n secs
heartbeat_conf      = 'OK_hrtbt'                    # heartbeat confirmation
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

logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)


### Establish TCP connections

client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
logging.info("TCP connection to GRC flowgraph open.")


if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")
    logging.warning("Custom msgs through serial not equal length. Sync might not work.")


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
            logging.info('Handshake confirmation recd -- saving data in ' +str(filename))
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
                    logging.debug('No stop_acq recd. Acquisition time out in ' +str(timeout) + ' seconds.')
                    acq_event.clear()
                    rospy.set_param('trigger/acknowledgement', True)
                    reset_buffer()
                    break
            end = time.time()
#                iocnt2 = psutil.disk_io_counters(perdisk=True)['/dev/nvme0n1p7']
            print(colored('\nFinished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'grey', 'on_green'))
            logging.info('Finished saving data in: ' +str(end - start) + ' seconds.')


def ros_events():
    """
    Check for ROS-based events like when the drone has reached WP, and trigger data acquisition.
    """
    if ser.isOpen() == True:
        print(ser, ser_timeout)
        reset_buffer()
        logging.info('Base serial is UP')
        send_telem(startup_initiate, ser, repeat_keyword)
        get_startup_confirmation = ser_timeout.read(msg_len*repeat_keyword)
        logging.debug('serial data: ' +str(get_startup_confirmation))
        if startup_confirm in get_startup_confirmation:
            print(colored('Communication to the payload is UP. Waiting for trigger from drone.', 'green'))
            logging.info('Comms to payload UP and RUNNING')
        else:
            print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
            logging.warning('Comms to payload DOWN')
    else:
        print(colored('No serial connection', 'magenta'))
        logging.warning('Base serial is DOWN')

    while not rospy.is_shutdown():
        sleep(1e-6)
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
            logging.info('Drone reached WP -- starting handshake')
            send_telem(handshake_start, ser, repeat_keyword)
            get_handshake_conf = ser_timeout.read(msg_len*repeat_keyword)
            logging.debug('serial data: ' +str(get_handshake_conf))
            if handshake_conf in get_handshake_conf:
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                send_telem(toggle_ON, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                print(get_stop_acq_trigger)
                logging.debug('serial data: ' +str(get_stop_acq_trigger))
                if toggle_OFF in get_stop_acq_trigger:
                    logging.info('Data acquisition toggled OFF')
                    acq_event.clear()
                    rospy.set_param('trigger/acknowledgement', True)
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                logging.warning('Handshake with payload failed. No data saved')
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
        send_telem(msg, ser, repeat_keyword)

        if msg == str(shutdown):
            print(colored('Shutting down payload and this code.', 'red'))
            logging.info("Manual kill switch. Shutting down payload and base station")
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
            pass

        elif msg == str(handshake_start):
            get_handshake_conf = ser_timeout.read(msg_len)
            print(get_handshake_conf)
            logging.info("Manual payload trigger")
            logging.debug("serial data: " +str(get_handshake_conf))
            if handshake_conf in get_handshake_conf:
                reset_buffer()
                print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                logging.info('Handshake confirmation recd -- acquiring data')
                send_telem(toggle_ON, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                print(get_stop_acq_trigger)
                logging.debug('serial data: ' +str(get_stop_acq_trigger))
                if toggle_OFF in get_stop_acq_trigger:
                    logging.info('Data acquisition toggled OFF')
                    acq_event.clear()
                    reset_buffer()
            else:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                logging.warning('Handshake with payload failed. No data saved')
                pass


def heartbeat():
    """
    Send telem heartbeat to ensure payload comms are okay.
    """
    while True:
        if not acq_event.is_set() and not rospy.get_param('trigger/command'):
            send_telem(heartbeat_check, ser, repeat_keyword)
            get_heartbeat = ser_timeout.read(msg_len)
            if heartbeat_conf in get_heartbeat:
                pass
            else:
                print(colored('Heartbeat not received from payload. Calibration MAY not work.', 'red'))
                logging.warning('Heartbeat NOT received. Calibration MAY not work. serial data: ' +str(get_heartbeat))
                pass
            reset_buffer()
            sleep(1)


def main():
    """
    Initiate threads.
    """
    try:
        t1 = Thread(target = recv_data)
        t2 = Thread(target = ros_events)
        t3 = Thread(target = manual_trigger_events)
        t4 = Thread(target = heartbeat)
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t1.join()
        t2.join()
        t3.join()
        t4.join()
    except (serial.SerialException, socket.error):
        print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')


if __name__ == '__main__':
    main()