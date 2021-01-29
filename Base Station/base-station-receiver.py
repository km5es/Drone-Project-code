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
#TODO: should there be a heartbeat thread/process as well to ensure that serial comms are working?
    #FIXME: Heartbeat feature is not working properly. Timing across threads is tricky. Tabling for now.
    #FIXME: One way to work around this would be to have long sleep durations and low timeouts. But that is 
    #FIXME: still risky because sometimes I get half a message which "folds" over. Hmm...
    #FIXME: got heartbeat over UDP not over telemetry for now.
#TODO: make it so the entire pipeline works over wi-fi AND telemetry. Is that possible?
    #FIXME: for now, I've added an argument which lets you choose betn wifi and telemetry.


import socket, serial, os, sys, rospy, logging, timeit, time, argparse
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
#pi                  = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
pi_addr             = "10.42.0.102"                 # default TCP address of payload
seq_port            = 6789
#xu4                 = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
xu4_addr            = "10.42.0.47"
hrt_beat_port       = 5678
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
pingtest            = 'pingtest'                    # manually test connection to payload
acq_event           = Event()                       # save radio data
timeout             = 4                             # time after which saving data will stop if no trigger
repeat_keyword      = 4                             # number of times to repeat a telem msg
ser                 = serial.Serial()               # dummy assignment in case no telemetry connected
ser_timeout         = serial.Serial()
network             = 'wifi'                        # options: wifi or telemtry 

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

if network == 'wifi':
    print(colored('Connecting to the drone via UDP', 'green'))

elif network == 'telemetry':
    print(colored('Connecting to the drone via ' + str(network), 'green'))


if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")
    logging.warning("Custom msgs through serial not equal length. Sync might not work.")


### Establish connections

try:
    ser         = serial.Serial('/dev/ttyPAYLOAD', 57600)
    ser_timeout = serial.Serial('/dev/ttyPAYLOAD', 57600, timeout=2)
    logging.info("Serial radio link established.")
except:
    print("No telemetry found. Check for Wi-Fi link...")
    logging.warning("No serial telemetry found")
    pass

# Connect to GRC flowgraph
client.connect((address))
print(colored('TCP connection to GRC opened on ' +str(address), 'green'))
logging.info("TCP connection to GRC flowgraph open.")

## UDP connection
# conn 1 for sync
payload_conn    = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
payload_conn.settimeout(4)
# conn 2 for sync
payload_conn2   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
payload_conn2.settimeout(4)

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
    if network == 'wifi':
        try:
            message, addr = payload_conn.recvfrom(msg_len*repeat_keyword)
            return message
        except (socket.timeout, TypeError):
            print(colored('Socket recv timed out in 4 seconds. Is the payload operatinal?', 'grey', 'on_red', attrs=['blink']))
            logging.debug('Socket recv timed out in 4 seconds. Is the payload operatinal?')
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
    else:
        print(colored('No serial connection', 'magenta'))
        logging.warning('Base serial is DOWN')

    send_telem(startup_initiate, ser, repeat_keyword)
#        get_startup_confirmation = ser_timeout.read(msg_len*repeat_keyword)
    try:
        get_startup_confirmation = recv_telem(msg_len, ser_timeout, repeat_keyword)
        logging.debug('serial data: ' +str(get_startup_confirmation))
        if startup_confirm in get_startup_confirmation:
            print(colored('Communication to the payload is UP. Waiting for trigger from drone.', 'green'))
            logging.info('Comms to payload UP and RUNNING')
        else:
            print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
            logging.warning('Comms to payload DOWN')
    except TypeError:
        print(colored('The payload is not responding. Please make sure it has been initiated.', 'red'))
        logging.warning('Comms to payload DOWN')

    while not rospy.is_shutdown():
        sleep(1e-6)
        if rospy.get_param('trigger/command'):
            rospy.set_param('trigger/command', False)
            rospy.set_param('trigger/acknowledgement', False)
            print(colored('Drone has reached waypoint. Initiating handshake with payload.', 'cyan'))
            logging.info('Drone reached WP -- starting handshake')
            send_telem(handshake_start, ser, repeat_keyword)
#            get_handshake_conf = ser_timeout.read(msg_len*repeat_keyword)
            try:
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                logging.debug('serial data: ' +str(get_handshake_conf))
                if handshake_conf in get_handshake_conf:
                    reset_buffer()
                    print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                    send_telem(toggle_ON, ser, repeat_keyword)
                    acq_event.set()
#                    get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                    get_stop_acq_trigger = recv_telem(msg_len, ser, repeat_keyword)
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
            except TypeError:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                logging.warning('Handshake with payload failed. No data saved')
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
#            get_handshake_conf = ser_timeout.read(msg_len)
            try:
                get_handshake_conf = recv_telem(msg_len, ser_timeout, repeat_keyword)
                print(get_handshake_conf)
                logging.info("Manual payload trigger")
                logging.debug("serial data: " +str(get_handshake_conf))
                if handshake_conf in get_handshake_conf:
                    reset_buffer()
                    print('Handshake confirmation recd from payload. Triggering calibration and saving data.')
                    logging.info('Handshake confirmation recd -- acquiring data')
                    send_telem(toggle_ON, ser, repeat_keyword)
                    acq_event.set()
#                    get_stop_acq_trigger = ser.read(msg_len*repeat_keyword)
                    get_stop_acq_trigger = recv_telem(msg_len, ser, repeat_keyword)
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
            except TypeError:
                print(colored('Handshake with drone comms failed. No data will be saved.', 'grey', 'on_red', attrs=['blink']))
                logging.warning('Handshake with payload failed. No data saved')
                pass
        
        elif msg == str(pingtest):
            try:
                get_return_ping = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if pingtest in get_return_ping:
                    print(colored('Ping reply recd from payload.', 'green'))
                    logging.info("Ping reply recd from payload.")
                else:
                    print(colored('Payload not responding to ping test.', 'red'))
                    logging.warning("Payload not responding to ping test.")
            except TypeError:
                print(colored('Payload not responding to ping test.', 'red'))
                logging.warning("Payload not responding to ping test.")


def heartbeat_telem():
    """
    Send telem heartbeat to ensure payload comms are okay.
    """
    if network == 'telemetry':
        while True:
            if not acq_event.is_set() and not rospy.get_param('trigger/command'):
                send_telem(heartbeat_check, ser, repeat_keyword)
#                get_heartbeat = ser_timeout.read(msg_len)
                get_heartbeat = recv_telem(msg_len, ser_timeout, repeat_keyword)
                if heartbeat_conf in get_heartbeat:
                    pass
                else:
                    print(colored('Heartbeat not received from payload. Calibration MAY not work.', 'red'))
                    logging.warning('Heartbeat NOT received. Calibration MAY not work. serial data: ' +str(get_heartbeat))
                    pass
                reset_buffer()
                sleep(1)


def heartbeat_udp():
    """
    Send heartbeat over UDP to ensure payload comms are okay. 
    """
    if network == 'wifi':
        while True:
            sleep(1)
            try:
                sendtime = time.time()
                payload_conn2.sendto(heartbeat_check, (pi_addr, hrt_beat_port))
                payload_conn2.sendto(heartbeat_check, (xu4_addr, hrt_beat_port))
                message, addr = payload_conn2.recvfrom(msg_len)
                recvtime = time.time()
                RTT = (recvtime - sendtime)*1000.0		# in ms
                RTT = round(RTT, 2)
                if heartbeat_conf in message:
                    #print('Heartbeat confirmation recd from server in {} ms'.format(RTT))
                    if RTT > 2000:
                        print(colored('RTT to payload is high: {} ms'.format(RTT), 'red'))
                        logging.warning('RTT to payload is high: {} ms'.format(RTT))
                else:
                    print(colored('No heartbeat received from payload', 'grey', 'on_red'))
                    logging.debug('No heartbeat received from payload')
            except:
                pass


def main():
    """
    Initiate threads.
    """
    try:
        t1 = Thread(target = recv_data)
        t2 = Thread(target = ros_events)
        t3 = Thread(target = manual_trigger_events)
        t4 = Thread(target = heartbeat_udp)
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