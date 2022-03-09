#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Tcp Toggle
# Generated: Wed Feb 16 20:42:31 2022
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

class tcp_toggle(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "Tcp Toggle")

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30.72e6/4
        self.wave_freq = wave_freq = samp_rate/8
        self.meas_freq = meas_freq = 1500e6
        self.min_buffer = min_buffer = 8200*512
        self.gain = gain = 0
        self.freq = freq = meas_freq - wave_freq

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("num_recv_frames=512, recv_frame_size=8200", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		otw_format='sc16',
        		channels=range(2),
        	),
        )
        self.uhd_usrp_source_0.set_clock_rate(30.72e6, uhd.ALL_MBOARDS)
        self.uhd_usrp_source_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0.set_subdev_spec('A:A A:B', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(gain, 0)
        self.uhd_usrp_source_0.set_center_freq(freq, 1)
        self.uhd_usrp_source_0.set_gain(0, 1)
        (self.uhd_usrp_source_0).set_min_output_buffer(4198400)
        self.blocks_interleave_0 = blocks.interleave(gr.sizeof_gr_complex*1, 1)
        self.blks2_tcp_sink_0 = grc_blks2.tcp_sink(
        	itemsize=gr.sizeof_gr_complex*1,
        	addr='127.0.0.1',
        	port=8800,
        	server=True,
        )



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_interleave_0, 0), (self.blks2_tcp_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_interleave_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.blocks_interleave_0, 1))

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_wave_freq(self.samp_rate/8)
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)

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
        self.uhd_usrp_source_0.set_gain(self.gain, 0)


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


if __name__ == '__main__':
    main()

