#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Tcp Toggle
# Generated: Fri Jul 31 19:18:30 2020
##################################################

from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import blks2 as grc_blks2
from optparse import OptionParser
import time
from threading import Thread
import socket
import rospy
from std_msgs.msg import Float32


class tcp_toggle(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Tcp Toggle")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 7.68e6*4
        self.min_buffer = min_buffer = 512*8200*2
        self.freq = freq = 150e6

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("num_recv_frames=512, recv_frame_size=8200", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		otw_format='sc16',
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0.set_subdev_spec('A:A', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(20, 0)
        (self.uhd_usrp_source_0).set_min_output_buffer(8396800)
        self.blks2_tcp_sink_0 = grc_blks2.tcp_sink(
        	itemsize=gr.sizeof_gr_complex*1,
        	addr='127.0.0.1',
        	port=8800,
        	server=True,
        )



        ##################################################
        # Connections
        ##################################################
        self.connect((self.uhd_usrp_source_0, 0), (self.blks2_tcp_sink_0, 0))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_source_0.set_center_freq(self.freq, 0)
        self.uhd_usrp_source_0.set_center_freq(self.freq, 1)

    def get_temp(self):
        return self.uhd_usrp_source_0.get_sensor('temp').to_real()


def main(top_block_cls=tcp_toggle, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."
    pub = rospy.Publisher('sdr_temperature', Float32, queue_size=10)
    rospy.init_node('SDR_temperature_node', anonymous=True)
    rate = rospy.Rate(5) # 5 Hz
    
    tb = top_block_cls()
    tb.start()

    while not rospy.is_shutdown():
        temp = tb.get_temp()
#        rospy.loginfo(temp)
        pub.publish(temp)
        rate.sleep()

    tb.stop()
    tb.wait()


def temp_over_socket(top_block_cls=tcp_toggle, options=None):
    """
    When a ROS event is set, this object will send temp data over a socket connection to its companion
    script.
    """
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    tb = top_block_cls()
    port    = 7890
    #s       = socket.socket()                       # Create a socket object
    #host    = socket.gethostbyname('127.0.0.1')     # Get local machine name
    #s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #s.bind((host, port))                            # Bind to the port
#   # while True:
    #s.listen(100)                                   # Now wait for client connection.
    #conn, addr = s.accept()
    #print(colored('TCP server listening for connection from GRC flowgraph.', 'green'))
    #print(colored('Connection to GRC flowgraph established on ' + str(addr), 'green'))
    #while conn is not False:
    #    conn.send(tb.get_temp)
    #    time.sleep(0.25)

if __name__ == '__main__':
    main()
#    t1 = Thread(target = main)
#    t2 = Thread(target = temp_over_socket)
#    t1.start()
#    t2.start()
#    t1.join()
#    t2.join()
