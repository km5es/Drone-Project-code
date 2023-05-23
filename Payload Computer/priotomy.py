'''
This will just be a module containing some of the old functions I wrote that are not being used currently.
Most of this code is from cal_seq and/or base-station-receiver.
Separate module because they may come in handy in the future and because I am not sure what to do with 
them as of now

author: Krishna Makhija
date: May 22nd 2023
'''

#? from cal_seq
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


#######################################################################################################


#? from base-station-receiver
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