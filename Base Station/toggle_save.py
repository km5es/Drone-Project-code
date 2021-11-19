#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Toggle Save
# Author: Krishna Makhija
# Description: This flowgraph will simply save data when the checkbox is ticked.
# Generated: Thu Nov 18 23:20:12 2021
##################################################

from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import time
import rospy
from std_msgs.msg import Float32

class toggle_save(gr.top_block):

    def __init__(self, timestamp=time.strftime("%H%M%S-%d%m%Y")):
        gr.top_block.__init__(self, "Toggle Save")

        ##################################################
        # Parameters
        ##################################################
        self.timestamp = timestamp

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 7.5e6
        self.wave_freq = wave_freq = samp_rate/3
        self.meas_freq = meas_freq = 150e6
        self.min_buffer = min_buffer = 512*8200*2
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
        self.uhd_usrp_source_0.set_clock_source('external', 0)
        self.uhd_usrp_source_0.set_subdev_spec('A:A A:B', 0)
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(gain, 0)
        self.uhd_usrp_source_0.set_center_freq(freq, 1)
        self.uhd_usrp_source_0.set_gain(0, 1)
        (self.uhd_usrp_source_0).set_min_output_buffer(8396800)
        self.blocks_interleave_0 = blocks.interleave(gr.sizeof_gr_complex*1, 1)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, '/mnt/78ACE633ACE5EB96/milton_raw_data/' +str(timestamp) + '_rx_data.dat', True)
        self.blocks_file_sink_0.set_unbuffered(False)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_interleave_0, 0), (self.blocks_file_sink_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blocks_interleave_0, 0))
        self.connect((self.uhd_usrp_source_0, 1), (self.blocks_interleave_0, 1))

    def get_timestamp(self):
        return self.timestamp

    def set_timestamp(self, timestamp):
        self.timestamp = timestamp

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

def argument_parser():
    description = 'This flowgraph will simply save data when the checkbox is ticked.'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--timestamp", dest="timestamp", type="string", default=time.strftime("%H%M%S-%d%m%Y"),
        help="Set 232009-18112021 [default=%default]")
    return parser


def main(top_block_cls=toggle_save, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    pub = rospy.Publisher('sdr_temperature', Float32, queue_size=10)
    rospy.init_node('SDR_temperature_node', anonymous=True)
    rate = rospy.Rate(10)   # 10 Hz

    tb = top_block_cls(timestamp=options.timestamp)
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
