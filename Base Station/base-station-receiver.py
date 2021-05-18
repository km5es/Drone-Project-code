#!/usr/bin/env python2.7
# client
"""
Base station code. Looks for when the drone has reached a waypoint and triggers payload to begin transmitting the
cal signal. There is also a concurrent thread that allows for manual initialization of the cal signal and/or
forced shutdown of the payload and base station. Logs are saved in the /logs directory of the repo.

Author: Krishna Makhija
date: 6th August 2020
last modified: 28th Jan 2021
"""

import socket, serial, os, sys, rospy, logging, timeit, time, argparse
#import psutil
from time import sleep
from termcolor import colored
from os.path import expanduser
from serial.serialutil import SerialException
from threading import Thread, Event
from random import randint

##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
#pi                  = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
pi_addr             = "10.42.0.102"                 # default TCP address of payload
seq_port            = 6789
#xu4                 = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
xu4_addr            = "10.42.0.47"
hrt_beat_port       = 5678
client_script_name  = 'tcp_toggle.py'               # TCP client name
path                = expanduser("~") + "/"         # define home path
logs_path           = path + 'catkin_ws/src/Drone-Project-code/logs/base/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_base_station_events.log") 
heartbeat_check     = 'hrt_beat'                    # heartbeat every n secs
heartbeat_conf      = 'OK_hrtbt'                    # heartbeat confirmation
startup_initiate    = 'pay_INIT'                    # check to see if payload is running
startup_confirm     = 'INITconf'                    # confirmation msg from payload if running
handshake_start     = 'is_comms'                    # begin handshake prior to save data
handshake_conf      = 'serialOK'                    # confirmation from payload before save
toggle_ON           = 'start_tx'                    # message to payload to start cal
toggle_OFF          = 'stop_acq'                    # message from payload to stop saving
stop_acq_conf       = 'confSTOP'                    # confirm that acquisition has stopped
shutdown            = 'shutdown'                    # force shutdown of all SDRs
startup_SDR         = 'beginSDR'                    # boot up SDR codes.
reboot_payload      = '_reboot_'                    # reboot payload computer
pingtest            = 'pingtest'                    # manually test connection to payload
update_wp           = 'updateWP'                    # manual update to WP table
restart_wp_node     = 'rswpnode'                            # manual reset of ROS WP nodes
acq_event           = Event()                       # save radio data
timeout             = 8                             # time after which saving data will stop if no trigger
repeat_keyword      = 64                             # number of times to repeat a telem msg
ser                 = serial.Serial()               # dummy assignment in case no telemetry connected
ser_timeout         = serial.Serial()
network             = 'telemetry'                        # options: wifi or telemtry 

logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)

### argparse

parser = argparse.ArgumentParser(description="Activate payload calibration when drone reaches WP. Manually trigger it by typing 'pay_INIT'")
parser.add_argument('-n', '--network', type=str, help='Choose type of communications. Options are telemetry or wifi.')
parser.add_argument('-a', '--address', type=str, help='IP address of TCP server (payload computer).')
parser.add_argument('-p', '--port', type=int, help='TCP port of the payload computer.')
args = parser.parse_args()

if args.network:
    network = args.network

if args.port:
    pi_port = args.port

if args.address:
    pi_addr = args.address

## Establish connections
if network == 'wifi':
    print(colored('Connecting to the drone via UDP', 'green'))
    ## UDP connection
    # conn 1 for sync
    payload_conn    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_conn.settimeout(timeout)
    # conn 2 for sync
    payload_conn2   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_conn2.settimeout(timeout)

elif network == 'telemetry':
    print(colored('Connecting to the drone via ' + str(network), 'green'))
    try:
        ser         = serial.Serial('/dev/ttyPAYLOAD', 57600)
        ser_timeout = serial.Serial('/dev/ttyPAYLOAD', 57600, timeout=2)
        print(colored("Serial radio link established to T960 payload.", "green"))
        logging.info("Serial radio link established to T960 payload.")
    except:
        try:
            ser         = serial.Serial('/dev/ttyF450', 57600)
            ser_timeout = serial.Serial('/dev/ttyF450', 57600, timeout=2)
            print(colored("Serial radio link established to F450 payload.", "green"))
            logging.info("Serial radio link established to F450 payload.")
        except:
            print(colored("No telemetry found. Check for Wi-Fi link...", "red"))
            logging.warning("No serial telemetry found")
            pass

if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")
    logging.warning("Custom msgs through serial not equal length. Sync might not work.")


# Connect to GRC flowgraph
client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
logging.info("TCP connection to GRC flowgraph open.")


### Define objects

def send_telem(keyword, serial_object, repeat_keyword):
    """
    Send keyword over telemetry radio or wireless connection for a total of repeat_keyword times.
    """
    for n in range(repeat_keyword):
        new_keyword = keyword + n*keyword
    if network == 'telemetry':
        serial_object.write(new_keyword)
    if network == 'wifi':
        try:
            payload_conn.sendto(new_keyword, (pi_addr, seq_port))
        except:
            pass
        try:
            payload_conn.sendto(new_keyword, (xu4_addr, seq_port))
        except:
            pass


def recv_telem(msg_len, serial_object, repeat_keyword):
    """
    Receive messages from the payload via telemetry or TCP.
    """
    if network == 'telemetry':
        message = serial_object.read(msg_len*repeat_keyword)
        return message
    if network == 'wifi':
        try:
            message, addr = payload_conn.recvfrom(msg_len*repeat_keyword)
            return message
        except (socket.timeout, TypeError):
            print(colored('Socket recv timed out in ' +str(timeout) + ' seconds. Is the payload operatinal?', 'grey', 'on_red', attrs=['blink']))
            logging.debug('Socket recv timed out in ' +str(timeout) + '  seconds. Is the payload operatinal?')
            pass


def reset_buffer():
    """
    Clear telemetry radio buffers whenever possible.
    """
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except serial.SerialException:
        pass


def get_timestamp():
    """
    Returns current time for data logging.
    """
    timestring = time.strftime("%H%M%S-%d%m%Y")
    return timestring


def heartbeat_udp():
    """
    Send heartbeat over UDP to ensure payload comms are okay. 
    """
    n = 4
    if network == 'wifi':
        while True:
            sleep(1)
            try:
                ping = 'ping' + ''.join(["{}".format(randint(0, 3)) for num in range(0, n)])
                sendtime = time.time()
                payload_conn2.sendto(ping, (pi_addr, hrt_beat_port))
                payload_conn2.sendto(ping, (xu4_addr, hrt_beat_port))
                message, addr = payload_conn2.recvfrom(msg_len)
                recvtime = time.time()
                RTT = (recvtime - sendtime)*1000.0		# in ms
                RTT = round(RTT, 2)
                #print(message)
                if ping in message:
                    #print('Heartbeat confirmation recd from server in {} ms'.format(RTT))
                    if RTT > 2000:
                        print(colored('RTT to payload is high: {} ms'.format(RTT), 'red'))
                        logging.warning('RTT to payload is high: {} ms'.format(RTT))
            except socket.timeout:
                print(colored('No heartbeat received from payload', 'grey', 'on_red'))
                logging.debug('No heartbeat received from payload')
                pass


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    global tel_flag
    while True:
        sleep(1e-6)

        if acq_event.is_set():
            print('Trigger from payload recd. Saving data now.')
            timestring      = get_timestamp()         
            filename        = path + timestring + str("_milton.dat")
            f               = open(filename, "w")
            print(colored('Saving data now in ' + str(filename), 'cyan'))
            logging.info('Handshake confirmation recd -- saving data in ' +str(filename))
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
                    reset_buffer()
                    break
            end = time.time()
            print(colored('\nFinished saving data in: ' +str(end - start) + ' seconds. Waiting for next waypoint.', 'grey', 'on_green'))
            logging.info('Finished saving data in: ' +str(end - start) + ' seconds.')


def serial_comms():
    '''
    Manually trigger payload and initiate saving data on base station.
    '''
    global sendtime
    while True:                                     
        msg = raw_input("Enter serial comms message here: ")        # send is_comms handshake request
        sendtime = time.time()
        send_telem(msg, ser, repeat_keyword)

        if msg == str(shutdown):
            print(colored('Shutting down payload and this code.', 'red'))
            logging.info("Manual kill switch. Shutting down payload and base station")
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
            pass
        
        elif msg == str(toggle_ON):
            print("Manually trigger payload cal.")
            logging.info("Manually trigger payload cal.")
            pass

        #elif msg == str(pingtest):
        #    try:
        #        get_return_ping = recv_telem(msg_len, ser_timeout, repeat_keyword)
        #        if pingtest in get_return_ping:
        #            recvtime = time.time()
        #            RTT = (recvtime - sendtime)*1000.0		# in ms
        #            RTT = round(RTT, 2)
        #            print(colored('Ping reply recd from payload in {} ms.'.format(RTT), 'green'))
        #            logging.info("Ping reply recd from payload.")
        #        else:
        #            print(colored('Payload not responding to ping test.', 'red'))
        #            logging.warning("Payload not responding to ping test.")
        #    except:
        #        pass

tel_flag = True

def get_trigger_from_drone():
    """
    This will start saving data when the drone has informed the base that it has reached 
    a WP. Moving forward, it will also save trigger the saving of SDR metadata.
    """
    #TODO: if send_telem has msg then stop recv_telem() briefly. implement while loop.
    global tel_flag
    while True:
#        if send_telem.event().clear():
        sleep(1e-6)
        try:
            get_handshake = recv_telem(msg_len, ser, repeat_keyword)
            if handshake_start in get_handshake:
                print(colored("Drone has reached WP, sending confirmation and beginning acquisition now.", "green"))
                logging.info("Drone has reached WP, sending confirmation and beginning acquisition now.")
                logging.debug("serial data: %s" %get_handshake)
                send_telem(handshake_conf, ser, repeat_keyword)
                acq_event.set()
                get_stop_acq_trigger = recv_telem(msg_len, ser, repeat_keyword)
                print(get_stop_acq_trigger)
                logging.debug('serial data: ' +str(get_stop_acq_trigger))
                if toggle_OFF in get_stop_acq_trigger:
                    logging.info('Data acquisition toggled OFF')
                    acq_event.clear()
                    send_telem(stop_acq_conf, ser, repeat_keyword)
                    reset_buffer()
            
            elif startup_initiate in get_handshake:
                print(colored("The payload is UP and RUNNING.", 'grey', 'on_green'))
                logging.info("The payload is UP and RUNNING.")
                reset_buffer()
            
            elif pingtest in get_handshake:
                recvtime = time.time()
                RTT = (recvtime - sendtime)*1000.0		# in ms
                RTT = round(RTT, 2)
                print(colored('Ping reply recd from payload in {} ms.'.format(RTT), 'green'))
                logging.info("Ping reply recd from payload.")
        except:
            pass


def main():
    """
    Initiate threads.
    """
    try:
        t1 = Thread(target = recv_data)
        t2 = Thread(target = get_trigger_from_drone)
        t3 = Thread(target = get_timestamp)
        #t4 = Thread(target = heartbeat_udp)
        t1.start()
        t2.start()
        t3.start()
        #t4.start()
        t1.join()
        t2.join()
        t3.join()
        #t4.join()
    except (serial.SerialException, socket.error):
        print(colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')


if __name__ == '__main__':
    main()