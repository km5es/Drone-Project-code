#!/usr/bin/env python2.7
# client
"""
Base station code. Back-end SDR code for saving all raw data during beam-mapping.
Also has a separate thread which will trigger phase cal from the payload.

Author: Krishna Makhija
date: 6th August 2020
last modified: 28th Jan 2022
"""

import socket, serial, os, rospy, logging, time, argparse
from time import sleep
from termcolor import colored
from os.path import expanduser
from serial.serialutil import SerialException
from threading import Thread, Event
from random import randint
from pynput import keyboard


##### Define global variables

client              = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
ip                  = socket.gethostbyname("127.0.0.1")
port                = 8800
address             = (ip,port)
#pi                  = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
pi_addr             = "10.42.1.102"                 # default TCP address of payload
seq_port            = 6789
#xu4                 = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
xu4_addr            = "10.42.0.47"
hrt_beat_port       = 5678
client_script_name  = 'tcp_toggle.py'               # TCP client name
### uncomment to save in home folder
home_path           = expanduser("~") + "/"         # define home path
### uncomment below to save on SSD
data_path           = '/mnt/78ACE633ACE5EB96/milton_raw_data/'
logs_path           = home_path + 'catkin_ws/src/Drone-Project-code/logs/base/'             
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
no_data_tx          = 'no__data'                    # tell base no cal was done here
shutdown            = 'shutdown'                    # force shutdown of all SDRs
startup_SDR         = 'beginSDR'                    # boot up SDR codes.
reboot_payload      = '_reboot_'                    # reboot payload computer
pingtest            = 'pingtest'                    # manually test connection to payload
update_wp           = 'updateWP'                    # manual update to WP table
restart_wp_node     = 'rswpnode'                    # manual reset of ROS WP nodes
begin_raw_beam      = 'beam_acq'                    # begin raw beam data acquisition
help_call           = 'helphelp'                    # help message for checking list of commands    
acq_event           = Event()                       # save radio data (long-term)
phase_cal_event     = Event()                       # save phase cal data
timeout             = 4                             # time after which saving data will stop if no trigger
repeat_keyword      = 4                             # number of times to repeat a telem msg
ser                 = serial.Serial()               # dummy assignment in case no telemetry connected
ser_timeout         = serial.Serial()
network             = 'wifi'                        # options: wifi or telemtry 
msg_len             = len(toggle_ON)
buff_size           = 8200 * 512                    # to match the device transport parameters on the TX SDR
logging.basicConfig(filename=log_name, 
                        format='%(asctime)s\t%(levelname)s\t{%(module)s}\t%(message)s', level=logging.DEBUG)

### argparse

parser = argparse.ArgumentParser(description="""Back-end SDR code for data acquisition. Press Ctrl + Alt + Q to begin
                                                saving data, and Ctrl + Alt + P to stop saving it. The SDR and LO
                                                will continue to run until the script is terminated.""")
parser.add_argument('-n', '--network', type=str, help='Choose type of communications. Options are telemetry or wifi.')
parser.add_argument('-a', '--address', type=str, help='IP address of UDP server (payload computer).')
parser.add_argument('-p', '--port', type=int, help='UDP port of the payload computer.')
args = parser.parse_args()

if args.network:
    network = args.network

if args.port:
    pi_port = args.port

if args.address:
    pi_addr = args.address


def get_timestamp():
    """
    Returns current time for data logging.
    """
    time_now = time.time()
    mlsec = repr(time_now).split('.')[1][:3]
    time_now = time.strftime("%H:%M:%S.{}-%d/%m/%Y".format(mlsec))
    return time_now


## Establish connections
if network == 'wifi':
    print('%s: ' %(get_timestamp()) + colored('Connecting to the drone via UDP', 'green'))
    ## UDP connection
    # conn 1 for sync
    payload_conn    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_conn.settimeout(timeout)
    # conn 2 for sync
    payload_conn2   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload_conn2.settimeout(timeout)

elif network == 'telemetry':
    print('%s: ' %(get_timestamp()) + colored('Connecting to the drone via ' + str(network), 'green'))
    try:
        ser         = serial.Serial('/dev/ttyPAYLOAD', 4800)
        ser_timeout = serial.Serial('/dev/ttyPAYLOAD', 4800, timeout=timeout)
        print('%s: ' %(get_timestamp()) + colored("Serial radio link established to T960 payload.", "green"))
        logging.info("Serial radio link established to T960 payload.")
    except:
        try:
            ser         = serial.Serial('/dev/ttyF450', 4800)
            ser_timeout = serial.Serial('/dev/ttyF450', 4800, timeout=timeout)
            print('%s: ' %(get_timestamp()) + colored("Serial radio link established to F450 payload.", "green"))
            logging.info("Serial radio link established to F450 payload.")
        except:
            print('%s: ' %(get_timestamp()) + colored("No serial telemetry found.", "red"))
            logging.warning("No serial telemetry found")
            pass

# Connect to GRC flowgraph
client.connect((address))
print('%s: ' %(get_timestamp()) + colored('TCP connection to GRC opened on ' +str(address), 'green'))
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
                        print('%s: ' %(get_timestamp()) + colored('RTT to payload is high: {} ms'.format(RTT), 'red'))
                        logging.warning('RTT to payload is high: {} ms'.format(RTT))
            except socket.timeout:
                print('%s: ' %(get_timestamp()) + colored('No heartbeat received from payload', 'grey', 'on_red'))
                logging.debug('No heartbeat received from payload')
                pass


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
                print('%s: ' %(get_timestamp()) + colored("Drone has reached WP, sending confirmation and beginning acquisition now.", "green"))
                logging.info("Drone has reached WP, sending confirmation and beginning acquisition now.")
                logging.debug("serial data: %s" %get_handshake)
                send_telem(handshake_conf, ser, repeat_keyword)
                reset_buffer()
                acq_event.set()
                rospy.set_param('trigger/metadata', True)
                # ! is the serial really timing out?
                get_stop_acq_trigger = recv_telem(msg_len, ser_timeout, repeat_keyword)
                print('%s: ' %(get_timestamp()) + str(get_stop_acq_trigger))
                logging.debug('serial data: ' +str(get_stop_acq_trigger))
                reset_buffer()
                if toggle_OFF in get_stop_acq_trigger:
                    print('%s: ' %(get_timestamp()) + 'Data acquisition togled OFF')
                    logging.info('Data acquisition toggled OFF')
                    acq_event.clear()
                    rospy.set_param('trigger/metadata', False)
                    send_telem(stop_acq_conf, ser, repeat_keyword)
                #TODO: add elif call for special stop_acq msg
                elif no_data_tx in get_stop_acq_trigger:
                    print('%s: ' %(get_timestamp()) + 'Handshaking failed on drone side. CAL was NOT performed. Discard WP data.')
                    logging.warning('Handshaking failed on drone side. CAL was NOT performed. Discard WP data.')
                    acq_event.clear()
                    rospy.set_param('trigger/metadata', False)
                    #send_telem(stop_acq_conf, ser, repeat_keyword)
            #TODO: add another elif call for special stop_acq msg for handshake start
            elif no_data_tx in get_handshake:
                print('%s: ' %(get_timestamp()) + 'Sequence error. CAL was not performed.')
                logging.debug('Sequence error. CAL was not performed.')
                reset_buffer()

            elif startup_initiate in get_handshake:
                print('%s: ' %(get_timestamp()) + colored("The payload is UP and RUNNING.", 'grey', 'on_green'))
                logging.info("The payload is UP and RUNNING.")
                reset_buffer()
            
            elif pingtest in get_handshake:
                recvtime = time.time()
                RTT = (recvtime - sendtime)*1000.0		# in ms
                RTT = round(RTT, 2)
                print('%s: ' %(get_timestamp()) + colored('Ping reply recd from payload in {} ms.'.format(RTT), 'green'))
                logging.info('Ping reply recd from payload in {} ms.'.format(RTT))
        except:
            pass


def recv_data():
    '''
    Wait for acq_event to begin and stop saving data.
    '''
    global tel_flag
    while True:
        sleep(1e-3)
        #? begin phase cal data acquisition
        if phase_cal_event.is_set():
            print('%s: ' %(get_timestamp()) + 'Trigger from payload recd. Saving data now.')
            timestring      = time.strftime("%H%M%S-%d%m%Y")         
            filename        = data_path + timestring + str("_phase_cal.dat")
            f               = open(filename, "w")
            print('%s: ' %(get_timestamp()) + colored('Saving data now in ' + str(filename), 'cyan'))
            logging.info('Handshake confirmation recd -- saving data in ' +str(filename))
            start           = time.time()
            start_timeout   = start + timeout            
            while True:
                SDRdata     = client.recv(buff_size, socket.MSG_WAITALL)
                f.write(SDRdata)
                if phase_cal_event.is_set() == False:
                    break               
                elif time.time() > start_timeout:
                    print('%s: ' %(get_timestamp()) + colored('No stop_acq message received from drone. Acquisition timed out in '\
                                                                            +str(timeout) + ' seconds.', 'grey', 'on_magenta'))
                    logging.debug('No stop_acq recd. Acquisition time out in ' +str(timeout) + ' seconds.')
                    phase_cal_event.clear()
                    rospy.set_param('trigger/metadata', False)
                    reset_buffer()
                    break
            end = time.time()
            print('%s: ' %(get_timestamp()) + colored('\nFinished saving data in: ' \
                                                            +str(end - start) + ' seconds.', 'grey', 'on_green'))
            logging.info('Finished saving data in: ' +str(end - start) + ' seconds.')
        #? begin beam data acquisition
        elif acq_event.is_set():
            timestring      = time.strftime("%H%M%S-%d%m%Y")         
            filename        = data_path + timestring + str("_milton.dat")
            f               = open(filename, "w")
            print('%s: ' %(get_timestamp()) + colored('Saving raw beam mapping data now in ' + str(filename), 'cyan'))
            logging.info('Saving raw beam mapping data in ' +str(filename))
            while True:
                SDRdata     = client.recv(buff_size, socket.MSG_WAITALL)
                f.write(SDRdata)
                if acq_event.is_set() == False:
                    print('%s: ' %(get_timestamp()) + colored('Raw beam data acquisition stopped.', 'red'))
                    logging.info('Raw beam data acquisition stopped.')
                    break  


def serial_comms_old():
    '''
    Manually trigger payload and initiate saving data on base station.
    #TODO: refactor this bit so I am not repeating steps
    '''
    global sendtime
    while True:                                     
        msg = raw_input("Enter serial comms message here (type 'helphelp' for list of commands): ")
        sendtime = time.time()
        #? shutdown GR codes on base and payload
        if msg == str(shutdown):
            try:
                send_telem(msg, ser, repeat_keyword)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if handshake_conf in get_handshake_conf:
                    print('%s: ' %(get_timestamp()) + colored('Shutting down payload and this code.', 'red'))
                    logging.info("Manual kill switch. Shutting down payload and base station")
                    reset_buffer()
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial connection. Payload will not be shutdown.', 'red'))
                logging.debug('No serial connection. Payload will not be shutdown.')
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
        #? begin phase cal
        elif msg == str(handshake_start):
            try:
                send_telem(msg, ser, repeat_keyword)
                print('%s: ' %(get_timestamp()) + "Manually trigger payload for phase calibration.")
                logging.info("Manually trigger payload cal.")
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if handshake_conf in get_handshake_conf:
                    send_telem(toggle_ON, ser, repeat_keyword)
                    phase_cal_event.set()
                    get_stop_acq = recv_telem(msg_len, ser_timeout, repeat_keyword)
                    if toggle_OFF in get_stop_acq:
                        phase_cal_event.clear()
                        reset_buffer()
                else:
                    print('%s: ' %(get_timestamp()) + "No handshake confirmation from payload. NO phase cal data saved.")
                    logging.debug("No handshake confirmation from payload. NO phase cal data saved.")
                    reset_buffer()
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial connection. Phase cal will not be initiated.', 'red'))
                logging.debug('No serial connection. Phase cal will not be initiated.')
        #? reset ROS nodes
        elif msg == str(restart_wp_node):
            try:
                send_telem(msg, ser, repeat_keyword)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if handshake_conf in get_handshake_conf:
                    print('%s: ' %(get_timestamp()) + "Payload has restarted ROS nodes.")
                    logging.info("Payload has restarted ROS nodes.")
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial connection. No action taken on the payload.', 'red'))
                pass
        #? ping test
        elif msg == str(pingtest):
            try:
                send_telem(msg, ser, repeat_keyword)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if heartbeat_conf in get_handshake_conf:
                    print('%s: ' %(get_timestamp()) + colored("Ping reply received from payload.", 'green'))
                    logging.debug("Ping reply received from payload.")
                else:
                    print('%s: ' %(get_timestamp()) + colored("No ping reply received from payload. Check serial connection.", 'red'))
                    logging.debug("No ping reply received from payload. Check serial connection.")
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial connection. No action taken on the payload.', 'red'))
                pass
        #? begin long-term data acquisition 
        elif msg == str(begin_raw_beam):
            acq_event.set()
        #? stop long-term data acquisition
        elif msg == str(toggle_OFF):
            acq_event.clear()
        #? help message for list of commands
        elif msg == str(help_call):
            print('%s: ' %(get_timestamp()) + colored("\n\nHere are the list of available commands:\n", 'white', attrs=['underline']) + 
                            colored("is_comms:  ", 'cyan') + "Begin phase cal.\n" +
                            colored("rswpnode:  ", 'cyan') + "Restart ROS nodes on payload (do this when WP table is to be updated).\n" +
                            colored("shutdown:  ", 'cyan') + "Kill all SDR codes on payload and base.\n" +
                            colored("pingtest:  ", 'cyan') + "Ping test to payload to verify comms.\n" +
                            colored("beam_acq:  ", 'cyan') + "Start raw data acquisition (Ctrl + Alt + Q).\n" +
                            colored("stop_acq:  ", 'cyan') + "Stop raw data acquisition (Ctrl + Alt + P).\n")


def serial_comms():
    '''
    Manually trigger payload and initiate saving data on base station.
    #FIXME: each subsequent phase cal is shorter in length. why?
    '''
    global sendtime
    while True:
        sleep(0.5)                                     
        msg = raw_input("Enter serial comms message here (type 'helphelp' for list of commands): ")
        sendtime = time.time()
        if msg == (handshake_start) or msg == (restart_wp_node) or msg == (pingtest):
            try:
                send_telem(msg, ser, repeat_keyword)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if handshake_conf in get_handshake_conf:
                #? begin phase cal
                    if msg == str(handshake_start):
                        print('%s: ' %(get_timestamp()) + "Manually trigger payload for phase calibration.")
                        logging.info("Manually trigger payload cal.")
                        send_telem(toggle_ON, ser, repeat_keyword)
                        phase_cal_event.set()
                        get_stop_acq = recv_telem(msg_len, ser_timeout, repeat_keyword)
                        if toggle_OFF in get_stop_acq:
                            phase_cal_event.clear()
                            reset_buffer()
                #? reset ROS nodes
                    elif msg == str(restart_wp_node):
                        print('%s: ' %(get_timestamp()) + "Payload has restarted ROS nodes.")
                        logging.info("Payload has restarted ROS nodes.")
                #? ping test
                elif msg == str(pingtest) and (heartbeat_conf in get_handshake_conf):
                    print('%s: ' %(get_timestamp()) + colored("Ping reply received from payload.", 'green'))
                    logging.debug("Ping reply received from payload.")
                else:
                    print('%s: ' %(get_timestamp()) + colored("No reply from payload on initial message: %s" %get_handshake_conf, 'magenta'))
                    logging.debug("No reply from payload on initial message.")
                    reset_buffer()
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial and/or UDP connection to the payload.', 'red'))
                logging.debug('No serial and/or UDP connection to the payload.')
        #? shutdown GR codes on base and payload
        elif msg == str(shutdown):
            try:
                send_telem(msg, ser, repeat_keyword)
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if handshake_conf in get_handshake_conf:
                    print('%s: ' %(get_timestamp()) + colored('Shutting down payload and this code.', 'red'))
                    logging.info("Manual kill switch. Shutting down payload and base station")
                    reset_buffer()
            except (serial.SerialException, TypeError):
                print('%s: ' %(get_timestamp()) + colored('No serial connection. Payload will not be shutdown.', 'red'))
                logging.debug('No serial connection. Payload will not be shutdown.')
            os.system('kill -9 $(pgrep -f ' +str(client_script_name) + ')')
            os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')
        #? begin long-term data acquisition 
        elif msg == str(begin_raw_beam):
            acq_event.set()
        #? stop long-term data acquisition
        elif msg == str(toggle_OFF):
            acq_event.clear()
        #? help message for list of commands
        elif msg == str(help_call):
            print('%s: ' %(get_timestamp()) + colored("\n\nHere are the list of available commands:\n", 'white', attrs=['underline']) + 
                            colored("is_comms:  ", 'cyan') + "Begin phase cal.\n" +
                            colored("rswpnode:  ", 'cyan') + "Restart ROS nodes on payload (do this when WP table is to be updated).\n" +
                            colored("shutdown:  ", 'cyan') + "Kill all SDR codes on payload and base.\n" +
                            colored("pingtest:  ", 'cyan') + "Ping test to payload to verify comms.\n" +
                            colored("beam_acq:  ", 'cyan') + "Start raw data acquisition (Ctrl + Alt + Q).\n" +
                            colored("stop_acq:  ", 'cyan') + "Stop raw data acquisition (Ctrl + Alt + P).\n")


def hk_begin_acq():
    acq_event.set()

def hk_cease_acq():
    acq_event.clear()

def input_hotkey():
    '''
    Add hotkeys for starting and stopping data acquisition.
    Ctrl + Alt + Q: begin acquisition
    Ctrl + Alt + P: cease acquisition
    '''
    with keyboard.GlobalHotKeys({
            '<alt>+<ctrl>+q': hk_begin_acq,
            '<alt>+<ctrl>+p': hk_cease_acq,}) as h:
        h.join()


def main():
    """
    Initiate threads.
    """
    try:
        t1 = Thread(target = recv_data)
        t2 = Thread(target = serial_comms)
        t3 = Thread(target = input_hotkey)
        t1.start()
        t2.start()
        t3.start()
        t1.join()
        t2.join()
        t3.join()
    except (serial.SerialException, socket.error):
        print('%s: ' %(get_timestamp()) + colored("Socket/serial device exception found. Killing processes and retrying...", 'red'))
        os.system('kill -9 $(fuser /dev/ttyUSB0)')
        os.system('lsof -t -i tcp:' +str(port) + ' | xargs kill -9')


if __name__ == '__main__':
    main()