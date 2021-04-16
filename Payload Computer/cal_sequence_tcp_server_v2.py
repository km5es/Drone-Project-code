#!/usr/bin/env python2.7
# server
'''
This script will combine with gr_cal_tcp_loopback_client.py to generate a cal signal from the drone. It reads from serial radio for
a trigger to begin the sequence. Once that is received, a TCP connection is established with the aforementioned GRC script, and a file
containing a predefined waveform is streamed over the TCP link. Once a certain number of pulses are reached, the RF switch is toggled
using GPIO. That number is defined as togglePoint and 2x togglePoint causes the sequence to finish.
In addition, some "magic" commands can be used to manually trigger calibration and shut down the codes. There is also a system reboot 
command. Logs are saved in the /logs directory of the repo.

Author: Krishna Makhija
date: 21st July 2020
last modified: 28th Jan 2021
v2.0 now will transmit zeros while the trigger is not set. This will ensure an integer number of cycles on the LO to complete before the
cal signal is transmitted. This ensures each transmission is phase consistent with the previous one.
'''
#TODO: integrate the circular polarized waveform as well.
#TODO: integrate GPIO w RF switch
#FIXME: reboot payload is not working

import socket, serial, os, sys, time, rospy, logging, argparse
from termcolor import colored
from datetime import datetime
from threading import Thread, Event
from os.path import expanduser


### Define global variables

togglePoint         = 96                                    # number of pulses after which GPIO is toggled
sample_packet       = 4096*16                               # Length of one pulse.
s                   = socket.socket()                       # Create a socket object
host                = socket.gethostbyname('127.0.0.1')     # Get local machine name
port                = 8810
#base_station        = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#base_station_ip     = socket.gethostbyname('0.0.0.0')
#base_station_port   = 12000
heartbeat_check     = 'hrt_beat'                            # heartbeat every n secs
heartbeat_conf      = 'OK_hrtbt'                            # heartbeat confirmation
startup_initiate    = 'pay_INIT'                            # check to see if payload is running
startup_confirm     = 'INITconf'                            # confirmation msg from payload if running
handshake_start     = 'is_comms'                            # begin handshake prior to save data
handshake_conf      = 'serialOK'                            # confirmation from payload before save
toggle_ON           = 'start_tx'                            # message to payload to start cal                
toggle_OFF          = 'stop_acq'                            # message from payload to stop saving
shutdown            = 'shutdown'                            # force shutdown of all SDRs
reboot_payload      = '_reboot_'                            # reboot payload computer
pingtest            = 'pingtest'                            # manually test connection
repeat_keyword      = 4
client_script_name  = 'gr_cal_tcp_loopback_client.py'
trigger_event       = Event()
stop_acq_event      = Event()
metadata_acq_time   = 10
path                = expanduser("~") + "/"         # define home path
logs_path           = path + '/Drone-Project-code/logs/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_payload_events.log")
network             = 'telemetry'
ser                 = serial.Serial()
ser_timeout         = serial.Serial()

logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)

### argparse

parser = argparse.ArgumentParser(description="Activate payload calibration when drone reaches WP. Manually trigger it by typing 'pay_INIT'")
parser.add_argument('-n', '--network', type=str, help='Choose type of communications. Options are telemetry or wifi.')
parser.add_argument('-p', '--port', type=int, help='TCP port of the payload computer.')
args = parser.parse_args()

if args.network:
    network = args.network

if args.port:
    base_station_port = args.port

if network == 'wifi':
    print(colored('Connecting to the drone via ' + str(network),  'green'))

elif network == 'telemetry':
    print(colored('Connecting to the drone via ' + str(network), 'green'))

### Make TCP and serial connections

try:
    ser                 = serial.Serial('/dev/ttyTELEM', 57600)  
    ser_timeout         = serial.Serial('/dev/ttyTELEM', 57600, timeout=2)
except:
    print("No telemetry found. Check for Wi-Fi link...")
    logging.warning("No serial telemetry found")
    pass

## connect to GRC flowgraph
os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))                                        # Bind to the port
s.listen(5)                                                 # Now wait for client connection.
conn, address = s.accept()
print(colored('TCP server listening for connection from GRC flowgraph.', 'green'))
logging.info("TCP server waiting for connection with GRC client flowgraph")
print(colored('Connection to GRC flowgraph established on ' + str(address), 'green'))
logging.info('Connection to GRC flowgraph established on ' + str(address))

if ser.isOpen() == True:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(colored('Serial connection to payload is UP. Waiting for trigger.', 'green'))
    logging.info('Payload serial is UP')
    print(ser)
else:
    print(colored('No serial connection', 'magenta'))
    logging.warning('Payload serial is DOWN')

if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")
    logging.warning("Custom msgs through serial not equal length. Sync might not work.")


### Define objects

def create_server():
    """
    Create UDP server for base station to connect to.
    """
    global base_conn
    global udp_ip
    global udp_port
    udp_ip          = socket.gethostname()
    udp_port        = 6789
    base_conn       = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    base_conn.settimeout(4)
    base_conn.bind((udp_ip, udp_port))


def send_telem(keyword, serial_object, repeat_keyword, addr):
    """
    Send keyword over telemetry radio or wireless connection for a total of repeat_keyword times.
    """
    for n in range(repeat_keyword):
        new_keyword = keyword + n*keyword
    if network == 'telemetry':
        serial_object.write(new_keyword)
    if network == 'wifi':
#        base_conn.send(new_keyword)
        base_conn.sendto(new_keyword, addr)


def recv_telem(msg_len, serial_object, repeat_keyword):
    """
    Receive messages from the payload via telemetry or TCP.
    """
    if network == 'telemetry':
        message = serial_object.read(msg_len*repeat_keyword)
        return message, "127.0.0.1"
    if network == 'wifi':
        try:
#            message = base_conn.recv(msg_len*repeat_keyword)
            message, addr = base_conn.recvfrom(msg_len*repeat_keyword)  # UDP server
            return message, addr
        except:
#            print(colored('Socket recv timed out in 4 seconds. Is the base in range?', 'grey', 'on_red', attrs=['blink']))
#            logging.debug('Socket recv timed out in 4 seconds. Is the base in range?')
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
            logging.info("Trigger from base recd. CAL ON")
            pulses = 0
            for pulses in range(togglePoint):
                conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint/2:
                    print(colored("Switching polarization now.", 'cyan')) ### replace with GPIO command
                    logging.info("Switching polarization now")
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            stop_acq_event.set()
            print(colored('Calibration sequence complete at GPS time: ' +str(timestamp_stop) + '. Total time taken was: ' + str(total_time) + ' seconds. Sending trigger to base and awaiting next trigger.', 'green'))
            logging.info("Cal sequence complete. CAL OFF")
            trigger_event.clear()


def sync_events():
    '''
    This object is for serial comms. When a handshake request is received, an event will be set in the stream_file() object which 
    will begin the calibration. When calibration is finished an event is set in stream_file() which sends a serial msg from this
    object to the base to stop acquisiiton.
    In addition, ROS flags will be set to begin and stop metadata generation.
    '''
    while True:
        create_server()
#        get_handshake = ser.read(msg_len*repeat_keyword)
        try:
            while True:
                get_handshake, addr = recv_telem(msg_len, ser, repeat_keyword)
                logging.debug("serial data: " +str(get_handshake))
                if handshake_start in get_handshake:
                    print(colored('Received handshake request from base station. Sending confirmation', 'cyan'))
                    logging.info("Received handshake from base. Sending confirmation")
                    send_telem(handshake_conf, ser, repeat_keyword, addr)
                    reset_buffer()
#                        get_trigger_from_base = ser_timeout.read(msg_len*repeat_keyword) ### set timeout here for handshake
                    get_trigger_from_base, addr = recv_telem(msg_len, ser_timeout, repeat_keyword)
                    logging.debug("serial data: " +str(get_trigger_from_base))
                    if toggle_ON in get_trigger_from_base:
                        trigger_event.set()
                        rospy.set_param('trigger/metadata', True)
                        while trigger_event.is_set() == True:
                            if stop_acq_event.is_set():
                                stop_acq_event.clear()
                                time.sleep(0.25)                                    ### buffer time for the receiver to "catch up".
                                send_telem(toggle_OFF, ser, repeat_keyword, addr)
                                reset_buffer()
                                time.sleep(metadata_acq_time)
                                rospy.set_param('trigger/metadata', False)
                    else:
                        print(colored('No start cal trigger recd from base. Waiting for next handshake request', 'magenta'))
                        logging.info("No start cal trigger from base")
                        pass
                    
                elif startup_initiate in get_handshake:
                    print(colored('The base has started up and is talking.', 'grey', 'on_green'))
                    logging.info("The base has started up and is talking")
                    send_telem(startup_confirm, ser, repeat_keyword, addr)
                    reset_buffer()
    
                elif heartbeat_check in get_handshake:
                    send_telem(heartbeat_conf, ser, repeat_keyword, addr)
                    reset_buffer()
    
                elif shutdown in get_handshake:
                    reset_buffer()
                    os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
                    os.system('lsof -t -i tcp:8810 | xargs kill -9')
                    print(colored('Kill command from base received. Shutting down TCP server and client programs.', 'red'))
                    logging.info("Manual kill command from base recd. Shutting down SDR code")
                    break
                
                elif reboot_payload in get_handshake:
                    reset_buffer()
                    print(colored('Rebooting payload', 'grey', 'on_red', attrs=['blink']))
                    logging.info(">>>REBOOTING PAYLOAD<<<")
                    os.system('sudo reboot now')

                elif get_handshake is None:
                    print('Connection to base broken.')
                    logging.debug('Connection to base broken.')
                    pass

                elif pingtest in get_handshake:
                    reset_buffer()
                    send_telem(pingtest, ser, repeat_keyword, addr)
                    print("Ping test received. Sending return ping.")
                    logging.info("Ping test received. Sending return ping.")

#        except socket.error:
#            print('Connection to base broken.')
#            logging.debug('Connection to base broken.')
#            pass
        except TypeError:
            pass


def heartbeat_udp():
    """
    Send heartbeat over UDP to ensure base connection is okay.
    """
    hrt_bt_port     = 5678
    base_conn2      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    base_conn2.bind((udp_ip, hrt_bt_port))
    if network == 'wifi':
        while True:
            time.sleep(0.01)
            data, addr = base_conn2.recvfrom(msg_len)
            if 'ping' in data:
                #print('Heartbeat received. Sending confirmation')
                base_conn2.sendto(data, addr)
                data = ""


def main():
    """
    Initiate threads.
    """
    t1 = Thread(target = sync_events)
    t2 = Thread(target = stream_file)
    t3 = Thread(target = heartbeat_udp)
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()


if __name__ == '__main__':
    main()