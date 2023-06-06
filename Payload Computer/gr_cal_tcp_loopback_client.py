#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: gr_cal_tcp_loopback_client
# Author: KM
# Description: This will go on the drone. A predefined waveform is fed into the companion script which creates a TCP server and loops back into this script. The server also checks for serial toggle and triggers GPIO at set points.
# Generated: Mon Jun  5 22:46:34 2023
##################################################

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import blks2 as grc_blks2
from optparse import OptionParser
import time
import rospy
from std_msgs.msg import Float32


class gr_cal_tcp_loopback_client(gr.top_block):

    def __init__(self, device_transport='send_frame_size=8192,  num_send_frames=512'):
        gr.top_block.__init__(self, "gr_cal_tcp_loopback_client")

        ##################################################
        # Parameters
        ##################################################
        self.device_transport = device_transport

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 7.68e6
        self.wave_freq = wave_freq = samp_rate/8
        self.meas_freq = meas_freq = 150e6
        self.min_buffer = min_buffer = 4096
        self.gain = gain = 40
        self.freq = freq = meas_freq - wave_freq

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
        	",".join(("", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_sink_0.set_clock_source('external', 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_center_freq(freq, 0)
        self.uhd_usrp_sink_0.set_gain(gain, 0)
        self.blocks_vector_to_stream_0 = blocks.vector_to_stream(gr.sizeof_gr_complex*1, min_buffer)
        (self.blocks_vector_to_stream_0).set_min_output_buffer(4096)
        self.blks2_tcp_source_0 = grc_blks2.tcp_source(
        	itemsize=gr.sizeof_gr_complex*min_buffer,
        	addr='127.0.0.1',
        	port=8810,
        	server=False,
        )
        (self.blks2_tcp_source_0).set_min_output_buffer(4096)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blks2_tcp_source_0, 0), (self.blocks_vector_to_stream_0, 0))
        self.connect((self.blocks_vector_to_stream_0, 0), (self.uhd_usrp_sink_0, 0))

    def get_device_transport(self):
        return self.device_transport

    def set_device_transport(self, device_transport):
        self.device_transport = device_transport

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_wave_freq(self.samp_rate/8)
        self.uhd_usrp_sink_0.set_samp_rate(self.samp_rate)

    def get_wave_freq(self):
        return self.wave_freq

    def set_wave_freq(self, wave_freq):
        self.wave_freq = wave_freq
        self.set_freq(self.meas_freq - self.wave_freq)

    def get_meas_freq(self):
        return self.meas_freq

    def set_meas_freq(self, meas_freq):
        self.meas_freq = meas_freq
        self.set_freq(self.meas_freq - self.wave_freq)

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain
        self.uhd_usrp_sink_0.set_gain(self.gain, 0)


    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_sink_0.set_center_freq(self.freq, 0)


def argument_parser():
    description = 'This will go on the drone. A predefined waveform is fed into the companion script which creates a TCP server and loops back into this script. The server also checks for serial toggle and triggers GPIO at set points.'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--device-transport", dest="device_transport", type="string", default='send_frame_size=8192,  num_send_frames=512',
        help="Set device_transport [default=%default]")
    return parser


def main(top_block_cls=gr_cal_tcp_loopback_client, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    pub = rospy.Publisher('sdr_temperature', Float32, queue_size=10)
    rospy.init_node('SDR_temperature_node', anonymous=True)
    rate = rospy.Rate(10)   # 10 Hz

    tb = top_block_cls(device_transport=options.device_transport)
    tb.start()
    while not rospy.is_shutdown():
        temp = tb.get_temp()
        pub.publish(temp)
        rate.sleep()
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
