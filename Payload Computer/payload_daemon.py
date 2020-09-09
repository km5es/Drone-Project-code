'''
This will be a daemon on the payload computer. It will receive telemetry commands from the base 
and initiate the SDR codes, stop and reboot them, and reboot the payload computer itself.

Author: Krishna Makhija
date: Sep 7th 2020
'''
import os
import serial
from termcolor import colored

### Define global variables

ser                 = serial.Serial('/dev/ttyUSB0', 57600)  
ser_timeout         = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)
startup_initiate    = 'pay_INIT'                            # check to see if payload is running
startup_confirm     = 'INITconf'                            # confirmation msg from payload if running
handshake_start     = 'is_comms'                            # begin handshake prior to save data
handshake_conf      = 'serialOK'                            # confirmation from payload before save
toggle_ON           = 'start_tx'                            # message to payload to start cal                
toggle_OFF          = 'stop_acq'                            # message from payload to stop saving
shutdown            = 'shutdown'                            # force shutdown of all SDRs
startup_SDR         = 'beginSDR'                            # boot up SDR codes.
reboot_payload      = '_reboot_'                            # reboot payload computer
test_hello          = 'hello_wd'
repeat_keyword      = 4
client_script_name  = 'gr_cal_tcp_loopback_client.py'


if len(toggle_ON) == len(toggle_OFF) == len(shutdown) == len(handshake_start) == len(handshake_conf):
    msg_len = len(toggle_ON)
else:
    raise Exception("Check custom messages to serial radios. Are they the right lengths?")


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


def main():
    """
    main function for telemetry commands from base.
    """ 
    if ser.isOpen() == True:
        reset_buffer()
        print(colored('Serial connection to payload is UP. Waiting for trigger.', 'green'))
        print(ser)
    while True:
        serial_msg = ser.read(msg_len*repeat_keyword)
        print(serial_msg)
        if startup_SDR in serial_msg:
            os.system('sh start_cal_v2.sh >> sdrlog.dat')
        elif reboot_payload in serial_msg:
            os.system('sudo reboot now')
        elif test_hello in serial_msg:
            os.system('python hello_world.py')


if __name__ == '__main__':
    main()