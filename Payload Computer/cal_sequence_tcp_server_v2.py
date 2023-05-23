#!/usr/bin/env python2.7
# server
'''
Control how and when the payload will transmit a calibration signal. When the drone 
reaches a WP, this script will trigger transmission of a pre-defined waveform. After 
a certain number of times that has happened (defined by togglePoint), this code will 
toggle the GPIO which controls the RF switch.

There is a separate mode wherein phase cal can be manually triggered from the base 
using either serial radios or a UDP connection.

Author: Krishna Makhija
date: 21st July 2020
last modified: Jan 28th 2022
'''

import socket, serial, os, time, rospy, logging, argparse
from termcolor import colored
from datetime import datetime
from threading import Thread, Event
from os.path import expanduser
import RPi.GPIO as GPIO


### Define global variables

togglePoint         = 96                                  # number of pulses per pol, each pulse is 0.065536s
sample_packet       = 4096*16                               # Length of one pulse.
s                   = socket.socket()                       # Create a socket object
host                = socket.gethostbyname('127.0.0.1')     # Get local machine name
port                = 8810
heartbeat_check     = 'hrt_beat'                            # heartbeat every n secs
heartbeat_conf      = 'OK_hrtbt'                            # heartbeat confirmation
startup_initiate    = 'pay_INIT'                            # check to see if payload is running
startup_confirm     = 'INITconf'                            # confirmation msg from payload if running
handshake_start     = 'is_comms'                            # begin handshake prior to save data
handshake_conf      = 'serialOK'                            # confirmation from payload before save
toggle_ON           = 'start_tx'                            # message to payload to start cal                
toggle_OFF          = 'stop_acq'                            # message from payload to stop saving
stop_acq_conf       = 'confSTOP'                            # confirm that acquisition has stopped
no_data_tx          = 'no__data'                            # tell base no cal was done here
shutdown            = 'shutdown'                            # force shutdown of all SDRs
reboot_payload      = '_reboot_'                            # reboot payload computer
pingtest            = 'pingtest'                            # manually test connection
update_wp           = 'updateWP'                            # manual update to WP table
restart_wp_node     = 'rswpnode'                            # manual reset of ROS WP nodes
repeat_keyword      = 4
client_script_name  = 'gr_cal_tcp_loopback_client.py'
trigger_event       = Event()
stop_acq_event      = Event()
serial_event        = Event()
metadata_acq_time   = 2
path                = expanduser("~") + "/"         # define home path
logs_path           = path + '/catkin_ws/src/Drone-Project-code/logs/payload/'             
log_name            = logs_path + time.strftime("%d-%m-%Y_%H-%M-%S_payload_events.log")
network             = 'wifi'
ser                 = serial.Serial()
ser_timeout         = serial.Serial()
timeout             = 4
wp_timeout          = 15
msg_len             = len(toggle_ON)

rospy.set_param('trigger/sequence', False)

### GPIO setup
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM)
GPIO_pin = 12   # 12 is circular, 16 is pol switch, 20 is Pol1 and 21 is Pol2
GPIO.setup (12, GPIO.OUT, initial=GPIO.LOW) # Set initial value
GPIO.setup (16, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)

logging.basicConfig(filename=log_name, format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)

### argparse
parser = argparse.ArgumentParser(description="Activate payload calibration when drone reaches WP. Manual trigger from base over UDP.")
parser.add_argument('-n', '--network', type=str, help='Choose type of communications. Options are telemetry or wifi.')
parser.add_argument('-p', '--port', type=int, help='UDP port of the payload computer.')
args = parser.parse_args()

if args.network:
    network = args.network

if args.port:
    base_station_port = args.port


def get_timestamp():
    """
    Returns current time for data logging.
    """
    time_now = time.time()
    mlsec = repr(time_now).split('.')[1][:3]
    time_now = time.strftime("%H:%M:%S.{}-%d/%m/%Y".format(mlsec))
    return time_now


### Make TCP and serial connections
print('%s: ' %(get_timestamp()) + colored('Connecting to the drone via ' + str(network),  'green'))
logging.info('Connecting to the drone via ' + str(network))
# Serial
try:
    ser                 = serial.Serial('/dev/ttyTELEM', 4800)  
    ser_timeout         = serial.Serial('/dev/ttyTELEM', 4800, timeout=timeout)
except:
    print('%s: ' %(get_timestamp()) + colored("No serial telemetry found.", 'magenta'))
    logging.warning("No serial telemetry found.")
    pass
if ser.isOpen() == True:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print('%s: ' %(get_timestamp()) + colored('Serial connection to payload is UP. Waiting for trigger.', 'green'))
    logging.info('Payload serial is UP')
    print(ser)
## connect to GRC flowgraph
os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((host, port))                                        # Bind to the port
s.listen(5)                                                 # Now wait for client connection.
conn, address = s.accept()
print('%s: ' %(get_timestamp()) + colored('TCP server listening for connection from GRC flowgraph.', 'green'))
logging.info("TCP server waiting for connection with GRC client flowgraph")
print('%s: ' %(get_timestamp()) + colored('Connection to GRC flowgraph established on ' + str(address), 'green'))
logging.info('Connection to GRC flowgraph established on ' + str(address))


### Define objects

def create_server():
    """
    Create UDP server for base station to connect to.
    """
    global server
    udp_port        = 6789
    server          = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind(("", udp_port))


def send_telem(keyword, serial_object, repeat_keyword, addr):
    """
    Send keyword over telemetry radio or wireless connection for a total of repeat_keyword times.
    """
    for n in range(repeat_keyword):
        new_keyword = keyword + n*keyword
    if network == 'telemetry':
        serial_object.write(new_keyword)
    if network == 'wifi':
        server.sendto(new_keyword, addr)


def recv_telem(msg_len, serial_object, repeat_keyword):
    """
    Receive messages from the payload via telemetry or UDP.
    """
    if network == 'telemetry':
        message = serial_object.read(msg_len*repeat_keyword)
        return message
    if network == 'wifi':
        try:
            message, addr = server.recvfrom(msg_len*repeat_keyword)
            return message, addr
        except (socket.timeout, TypeError):
            print('%s: ' %(get_timestamp()) + colored('Socket recv timed out in ' \
                            +str(timeout) + ' seconds. Is the payload operatinal?', 'grey', 'on_red', attrs=['blink']))
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


def stream_file():
    """
    Stream file to GRC code. Begin calibration when /trigger/sequence is set
    OR
    when there is a manual trigger from the base for phase cal.
    """
    zeros_wf         = 'zeros'
    zeros            = open(zeros_wf, 'rb')
    condition_LO     = zeros.read()
    cal_wf           = 'sine_waveform_pulsed'
    cal              = open(cal_wf,'rb')
    cal_signal       = cal.read()
    phase_cal_wf     = 'noise'
    phase_cal        = open(phase_cal_wf, 'rb')
    phase_cal_signal = phase_cal.read()

    n = 0
    while True:
        conn.send(condition_LO)
        #? transmit cal signal when WP is reached (pulsed sine)
        if rospy.get_param('trigger/sequence') == True:
            rospy.set_param('trigger/sequence', False)
            rospy.set_param('trigger/metadata', True)
            start = time.time()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            print('%s: ' %(get_timestamp()) + colored('Drone has reached WP at GPS time: ' \
                            +str(timestamp_start) + '. Beginning cal sequence using ' +str(cal_wf), 'green'))
            logging.info("Drone has reached WP. Beginning cal sequence using %s" %cal_wf)
            pulses = 0
            GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup (21, GPIO.OUT, initial=GPIO.HIGH)
            for pulses in range(togglePoint * 2):
                conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint:
                    GPIO.setup (20, GPIO.OUT, initial=GPIO.HIGH)
                    GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)
                    print('%s: ' %(get_timestamp()) + colored("Switching polarization now.", 'cyan'))
                    logging.info("Switching polarization now")
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            print('%s: ' %(get_timestamp()) + colored('Calibration sequence complete at GPS time: ' \
                            +str(timestamp_stop) + '. Total time taken was: ' + str(total_time) + ' seconds.', 'green'))
            logging.info("Cal sequence complete in %s seconds. CAL OFF" %total_time)
            rospy.set_param('trigger/metadata', False)
            rospy.set_param('trigger/waypoint', True) 
            GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)
        ##? manually begin phase cal using SSH telemetry
        if trigger_event.is_set():
            start = time.time()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            print('%s: ' %(get_timestamp()) + colored('Trigger from base received. Beginning phase cal sequence using ' \
                                                            +str(phase_cal_wf), 'green'))
            logging.info("Trigger from base recd. CAL ON")
            pulses = 0
            GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup (21, GPIO.OUT, initial=GPIO.HIGH)
            for pulses in range(togglePoint * 2):
                conn.send(cal_signal)                   # should this be noise or just a sine wave?
                pulses += 1
                if pulses == togglePoint:
                    GPIO.setup (20, GPIO.OUT, initial=GPIO.HIGH)
                    GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)
                    print('%s: ' %(get_timestamp()) + colored("Switching polarization now.", 'cyan'))
                    logging.info("Switching polarization now")
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            trigger_event.clear()
            print('%s: ' %(get_timestamp()) + colored('Calibration sequence complete. Total time taken was: ' \
                                                                + str(total_time) + ' seconds.', 'green'))
            logging.info("Cal sequence complete. CAL OFF")
            GPIO.setup(20, GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(21, GPIO.OUT, initial=GPIO.LOW)
            send_telem(toggle_OFF, ser, repeat_keyword, addr)
            reset_buffer()


def serial_comms_phase():
    """
    When manual trigger from base is received this will start phase cal.
    Phase cal will consist of a noise signal which is different from the 
    beam calibration signal.
    """
    global addr
    create_server()
    while True:
        time.sleep(0.05)
        try:
            get_handshake_from_base, addr = recv_telem(msg_len, ser, repeat_keyword)
            #? begin phase cal
            if handshake_start in get_handshake_from_base:
                print('%s: ' %(get_timestamp()) + "Handshake start for phase cal received from base.")
                logging.info("Handshake start for phase cal received from base.")
                send_telem(handshake_conf, ser, repeat_keyword, addr)
                get_start_acq, addr = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if toggle_ON in get_start_acq:
                    print('%s: ' %(get_timestamp()) + "Starting phase cal now.")
                    logging.info("Starting phase cal now.")
                    trigger_event.set()
            #? reset ROS nodes
            elif restart_wp_node in get_handshake_from_base:
                send_telem(handshake_conf, ser, repeat_keyword, addr)
                print('%s: ' %(get_timestamp()) + "Base has initiated manual reset of ROS nodes.")
                logging.info("Base has initiated manual reset of ROS nodes.")
                os.system('rosnode kill $(rosnode list | grep /write_WP)')
                os.system('rosnode kill $(rosnode list | grep /wp_trigger)')
                os.system('rosrun beam_mapping write_WPs.py &')
                os.system('rosrun beam_mapping wp_trigger.py &')
                reset_buffer()
            #? shutdown GR codes
            elif shutdown in get_handshake_from_base:
                send_telem(handshake_conf, ser, repeat_keyword, addr)
                print('%s: ' %(get_timestamp()) + colored('Kill command from base received. '\
                                            'Shutting down TCP server and client programs.', 'red'))
                logging.info("Manual kill command from base recd. Shutting down SDR code")
                reset_buffer()
                os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
                os.system('lsof -t -i tcp:8810 | xargs kill -9')
                break
            #? ping test
            elif pingtest in get_handshake_from_base:
                print('%s: ' %(get_timestamp()) + "Ping received from base. Sending reply...")
                logging.info("Ping received from base. Sending reply...")
                send_telem(heartbeat_conf, ser, repeat_keyword, addr)
                reset_buffer()
        except (serial.SerialException, TypeError):
            pass


def main():
    """
    Initiate threads.
    """
    rospy.init_node('cal_sequence', anonymous = True)
    t1 = Thread(target = serial_comms_phase)
    t2 = Thread(target = stream_file)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


if __name__ == '__main__':
    main()