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

togglePoint         = 96                                    # number of pulses per pol
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

'''
def begin_sequence():
    """
    This object will initiate the calibration sequence when a WP is reached.
    The WP is detected by wp_trigger.py
    When a WP is reached, the base station receiver will start recording data. 
    At the same time, metadata will be saved using get_metadata.py and trigger/metadata
    When the sequence is completed another ROS flag trigger/waypoint
    NOTE: I am just using telemetry for now.
    """
    create_server()
    addr = "127.0.0.1"
    send_telem(startup_initiate, ser, repeat_keyword, addr)
    reset_buffer()
    while not rospy.is_shutdown():
        # ? something is still weird about this. why does it retry so often? does the sleep time need adjustment?
        try:
            time.sleep(0.05)
            if rospy.get_param('trigger/sequence') == True:
                serial_event.set()      # stop other thread from recv telem
                for retry in range(1,4):
                    print(colored("Drone has reached WP. Sending handshake to base to begin acquisition.", 'cyan'))
                    logging.info("Drone has reached WP. Sending handshake to base to begin acquisition.")
                    send_telem(handshake_start, ser, repeat_keyword, addr)
                    get_handshake_conf, addr = recv_telem(msg_len, ser_timeout, repeat_keyword)
                    logging.debug("Serial data: %s" %get_handshake_conf)
                    reset_buffer()
                    if handshake_conf in get_handshake_conf:
                        print(colored("Handshake confirmation received from base. Beginning calibration sequence, and saving metadata.", 'green'))
                        logging.info("Handshake confirmation received from base. Beginning calibration sequence, and saving metadata.")
                        #rospy.set_param('trigger/sequence', False)
                        rospy.set_param('trigger/metadata', True)
                        start_time = time.time()
                        trigger_event.set()
                        while trigger_event.is_set() == True:
                            if stop_acq_event.is_set():
                                stop_acq_event.clear()
                                time.sleep(0.25)                                    ### buffer time for the receiver to "catch up".
                                send_telem(toggle_OFF, ser, repeat_keyword, addr)
                                get_stop_conf, addr = recv_telem(msg_len, ser_timeout, repeat_keyword)
                                if stop_acq_conf in get_stop_conf:
                                    reset_buffer()
                                    rospy.set_param('trigger/metadata', False)
                                    #rospy.set_param('trigger/waypoint', True)
                                    print('Base has stopped acquisition. Sequence complete.')
                                    logging.info('Base has stopped acquisition. Sequence complete.')
                                    serial_event.clear()    # allow other thread to recv telem
                                else:
                                    print('No stop acq confirmation from base. serial data: %s' %get_stop_conf)
                                    logging.debug('No stop acq confirmation from base. serial data: %s' %get_stop_conf)
                            elif time.time() >= start_time + wp_timeout:
                                print(colored("Sequence timeout out in %s seconds. Updating WP table and stopping metadata acq.", "red") %wp_timeout)
                                logging.warning("Sequence timeout out in %s seconds. Updating WP table and stopping metadata acq." %wp_timeout)
                                rospy.set_param('trigger/metadata', False)
                                #rospy.set_param('trigger/waypoint', True)
                                serial_event.clear()    # allow other thread to recv telem
                        break

                    else:
                        print("No handshake confirmation from base. Retry attempt #: %s" %retry)
                        logging.warning("No handshake confirmation from base. Retry attempt #: %s" %retry)
                        # * retry feature very useful * #
                if rospy.get_param('trigger/sequence') == True:
                    print(colored("Handshake with base station failed after 3 attempts. Moving to next WP now.", "red"))
                    logging.debug("Handshake with base station failed after 3 attempts. Moving to next WP now.")
                    #rospy.set_param('trigger/sequence', False)
                    #rospy.set_param('trigger/waypoint', True)
                    serial_event.clear()    # allow other thread to recv telem
                rospy.set_param('trigger/waypoint', True)
        except serial.SerialException, e:
            pass


def serial_comms():
    """
    This object is to maintain serial communications with the drone. It allows one to 
    switch the code on and off and maybe even reboot the payload computer.
    It also has a pingtest feature which ensures things are running.
    """
    while True:
        time.sleep(0.05)
        if rospy.get_param('trigger/sequence') == False and serial_event.is_set() == False:
            try:
                get_handshake, addr = recv_telem(msg_len, ser, repeat_keyword)
                logging.debug("serial data: " +str(get_handshake))
                # ! currently disabled.
                if startup_initiate in get_handshake:
                    print(colored('The base has started up and is talking.', 'grey', 'on_green'))
                    logging.info("The base has started up and is talking")
                    send_telem(startup_confirm, ser, repeat_keyword, addr)
                    reset_buffer()
                # * working okay
                elif shutdown in get_handshake:
                    os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
                    os.system('lsof -t -i tcp:8810 | xargs kill -9')
                    print(colored('Kill command from base received. Shutting down TCP server and client programs.', 'red'))
                    logging.info("Manual kill command from base recd. Shutting down SDR code")
                    reset_buffer()
                    break
                # ? no idea how this is doing
                elif reboot_payload in get_handshake:
                    print(colored('Rebooting payload', 'grey', 'on_red', attrs=['blink']))
                    logging.info(">>>REBOOTING PAYLOAD<<<")
                    os.system('sudo reboot now')
                    reset_buffer()
                # * this seems to work but does not print an exception is ping is missed
                elif pingtest in get_handshake:
                    print("Ping test received. Sending return ping.")
                    logging.info("Ping test received. Sending return ping.")
                    #serial_event.set()
                    send_telem(pingtest, ser, repeat_keyword, addr)
                    reset_buffer()
                # ? this works but the new WP does not meet the yaw cond
                elif update_wp in get_handshake:
                    print("Base station command to update WP table.")
                    logging.info("Base station command to update WP table.")
                    rospy.set_param('trigger/waypoint', True)
                    reset_buffer()
                # * working okay now
                elif restart_wp_node in get_handshake:
                    print("Resetting write_WPs.py and wp_trigger.py")
                    logging.info("Resetting write_WPs.py and wp_trigger.py")
                    os.system('pkill -f write_WPs.py')
                    os.system('pkill -f wp_trigger.py')
                    os.system('rosrun beam_mapping write_WPs.py &')
                    os.system('rosrun beam_mapping wp_trigger.py &')
                    reset_buffer()
                #FIXME: this will also cause WP to be updated. fix later.
                elif toggle_ON in get_handshake:
                    print('Manual trigger of sequence from base station.')
                    logging.info('Manual trigger of sequence from base station.')
                    rospy.set_param('trigger/sequence', True)
                    reset_buffer()
            except:
                pass


def begin_sequence_simple():
    """
    This object will initiate the calibration sequence when a WP is reached.
    The WP is detected by wp_trigger.py
    When a WP is reached, the base station receiver will start recording data. 
    At the same time, metadata will be saved using get_metadata.py and trigger/metadata
    When the sequence is completed another ROS flag trigger/waypoint
    NOTE: I am just using telemetry for now.
    """
    create_server()
    addr = "127.0.0.1"
    send_telem(startup_initiate, ser, repeat_keyword, addr)
    reset_buffer()
    while not rospy.is_shutdown():
        # ? something is still weird about this. there should be no retry if first handshake fails.
        try:
            time.sleep(0.01)
            if rospy.get_param('trigger/sequence') == True:
                rospy.set_param('trigger/sequence', False)
                print('%s: ' %(get_timestamp()) + colored("Drone has reached WP. Sending handshake to base to begin acquisition.", 'cyan'))
                logging.info("Drone has reached WP. Sending handshake to base to begin acquisition.")
                send_telem(handshake_start, ser, repeat_keyword, addr)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                logging.debug("Serial data: %s" %get_handshake_conf)
                reset_buffer()
                if handshake_conf in get_handshake_conf:
                    print('%s: ' %(get_timestamp()) + colored("Handshake confirmation received from base. Beginning calibration sequence, and saving metadata.", 'green'))
                    logging.info("Handshake confirmation received from base. Beginning calibration sequence, and saving metadata.")
                    #rospy.set_param('trigger/sequence', False)
                    rospy.set_param('trigger/metadata', True)
                    start_time = time.time()
                    trigger_event.set()
                    while trigger_event.is_set() == True:
                        time.sleep(0.1)
                        if stop_acq_event.is_set():
                            stop_acq_event.clear()
                            #time.sleep(0.25)                                    ### buffer time for the receiver to "catch up".
                            send_telem(toggle_OFF, ser, repeat_keyword, addr)
                            get_stop_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                            reset_buffer()
                            if stop_acq_conf in get_stop_conf:
                                print('%s: ' %(get_timestamp()) + 'Base has stopped acquisition. Sequence complete.')
                                logging.info('Base has stopped acquisition. Sequence complete.')
                                rospy.set_param('trigger/metadata', False)
                                rospy.set_param('trigger/waypoint', True)
                            else:
                                print('%s: ' %(get_timestamp()) + 'No stop acq confirmation from base. serial data: %s' %get_stop_conf)
                                logging.debug('No stop acq confirmation from base. serial data: %s' %get_stop_conf)
                                rospy.set_param('trigger/metadata', False)
                                rospy.set_param('trigger/waypoint', True)
                                serial_event.clear()

                #TODO: if no handshake then send special msg to stop acq so base knows no cal
                else:
                    send_telem(no_data_tx, ser, repeat_keyword, addr)
                    print('%s: ' %(get_timestamp()) + "No handshake confirmation from base. CAL was NOT performed.")
                    logging.warning("No handshake confirmation from base. CAL was NOT performed.")
                    rospy.set_param('trigger/waypoint', True)
                    reset_buffer()
                #rospy.set_param('trigger/waypoint', True)
        except serial.SerialException, e:
            pass


def stream_file_no_telem_pol_switch():
    """
    Stream file to GRC code. Begin calibration when /trigger/sequence is set.
    This function is used when there is to be no telemetry sync with base.
    """
    zeros = open('zeros', 'rb')
    condition_LO = zeros.read()
    filename = 'sine_waveform'
    f = open(filename,'rb')
    cal_signal = f.read()
    while True:
        conn.send(condition_LO)

        if rospy.get_param('trigger/sequence') == True:
            rospy.set_param('trigger/sequence', False)
            rospy.set_param('trigger/metadata', True)
            start = time.time()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            print('%s: ' %(get_timestamp()) + colored('Drone has reached WP at GPS time: ' +str(timestamp_start) + '. Beginning cal sequence using ' +str(filename), 'green'))
            logging.info("Drone has reached WP. Beginning cal sequence using %s" %filename)
            pulses = 0
            GPIO.output(12, GPIO.HIGH)          # circular first
            for pulses in range(togglePoint * 3):
                conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint:
                    GPIO.output(12, GPIO.LOW)
                    GPIO.output(16, GPIO.HIGH)  # linear pol
                    GPIO.output(20, GPIO.HIGH)  # pol 1
                    print('%s: ' %(get_timestamp()) + colored("Switching polarization now.", 'cyan')) ### replace with GPIO command
                    logging.info("Switching polarization now")
                if pulses == 2*togglePoint:
                    print("2/3rd point reached.")
                    GPIO.output(20, GPIO.LOW)
                    GPIO.output(21, GPIO.HIGH)  # pol 2
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            print('%s: ' %(get_timestamp()) + colored('Calibration sequence complete at GPS time: ' +str(timestamp_stop) + '. Total time taken was: ' + str(total_time) + ' seconds. Sending trigger to base and awaiting next trigger.', 'green'))
            logging.info("Cal sequence complete in %s seconds. CAL OFF" %total_time)
            rospy.set_param('trigger/metadata', False)
            rospy.set_param('trigger/waypoint', True) 
            GPIO.output(16, GPIO.LOW)
            GPIO.output(20, GPIO.LOW)
            GPIO.output(21, GPIO.LOW)
            GPIO.output(12, GPIO.LOW)
'''

def stream_file():
    """
    Stream file to GRC code. Begin calibration when /trigger/sequence is set
    OR
    when there is a manual trigger from the base for phase cal.
    """
    zeros_wf         = 'zeros'
    zeros            = open(zeros_wf, 'rb')
    condition_LO     = zeros.read()
    cal_wf           = 'sine_waveform'
    cal              = open(cal_wf,'rb')
    cal_signal       = cal.read()
    phase_cal_wf     = 'noise'
    phase_cal        = open(phase_cal_wf, 'rb')
    phase_cal_signal = phase_cal.read()

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
            GPIO.setup (20, GPIO.OUT, initial=GPIO.HIGH)
            GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)
            #GPIO.output(20, GPIO.HIGH)
            #GPIO.output(21, GPIO.LOW)
            for pulses in range(togglePoint * 2):
                conn.send(cal_signal)
                pulses += 1
                if pulses == togglePoint:
                    GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
                    GPIO.setup (21, GPIO.OUT, initial=GPIO.HIGH)
                    #GPIO.output(20, GPIO.LOW)
                    #GPIO.output(21, GPIO.HIGH)
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
        ##? transmit phase cal signal (noise)
        if trigger_event.is_set():
            start = time.time()
            timestamp_start = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            print('%s: ' %(get_timestamp()) + colored('Trigger from base received. Beginning phase cal sequence using ' \
                                                            +str(phase_cal_wf), 'green'))
            logging.info("Trigger from base recd. CAL ON")
            pulses = 0
            GPIO.setup (20, GPIO.OUT, initial=GPIO.HIGH)
            GPIO.setup (21, GPIO.OUT, initial=GPIO.LOW)
            #GPIO.output(20, GPIO.HIGH)
            #GPIO.output(21, GPIO.LOW)
            for pulses in range(togglePoint * 2):
                conn.send(phase_cal_signal)
                pulses += 1
                if pulses == togglePoint:
                    GPIO.setup (20, GPIO.OUT, initial=GPIO.LOW)
                    GPIO.setup (21, GPIO.OUT, initial=GPIO.HIGH)
                    #GPIO.output(20, GPIO.LOW)
                    #GPIO.output(21, GPIO.HIGH)
                    print('%s: ' %(get_timestamp()) + colored("Switching polarization now.", 'cyan'))
                    logging.info("Switching polarization now")
            timestamp_stop = datetime.now().strftime("%H:%M:%S.%f-%d/%m/%y")
            end = time.time()
            total_time = end - start
            #stop_acq_event.set()
            trigger_event.clear()
            print('%s: ' %(get_timestamp()) + colored('Calibration sequence complete. Total time taken was: ' \
                                                                + str(total_time) + ' seconds.', 'green'))
            logging.info("Cal sequence complete. CAL OFF")
            #GPIO.output(20, GPIO.LOW)
            #GPIO.output(21, GPIO.LOW)
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
                #    #while trigger_event.is_set() == True:
                #        time.sleep(0.05)
                #        #if stop_acq_event.is_set():
                #            #trigger_event.clear()
                #            #stop_acq_event.clear()
                #            print('%s: ' %(get_timestamp()) + "Stopping phase cal now.")
                #            logging.info("Stopping phase cal now.")
                #            send_telem(toggle_OFF, ser, repeat_keyword, addr)
                #            reset_buffer()
            #? reset ROS nodes
            elif restart_wp_node in get_handshake_from_base:
                send_telem(handshake_conf, ser, repeat_keyword, addr)
                print('%s: ' %(get_timestamp()) + "Base has initiated manual reset of ROS nodes.")
                logging.info("Base has initiated manual reset of ROS nodes.")
                os.system('rosnode kill $(rosnode list | grep /write_WP)')
                os.system('rosnode kill $(rosnode list | grep /wp_trigger)')
                #os.system("rosnode kill $(rosnode list | grep '/write_WP\|/wp_trigger')")
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