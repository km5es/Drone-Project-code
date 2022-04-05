#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: gr_cal_tcp_loopback_client
# Author: KM
# Description: This will go on the drone. A predefined waveform is fed into the companion script which creates a TCP server and loops back into this script. The server also checks for serial toggle and triggers GPIO at set points.
# GNU Radio version: 3.8.1.0

from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time

class gr_cal_tcp_loopback_client(gr.top_block):

    def __init__(self, device_transport=send_frame_size=8192,  num_send_frames=512):
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
        self.min_buffer = min_buffer = 65536
        self.gain = gain = 60
        self.freq = freq = meas_freq - wave_freq

        ##################################################
        # Blocks
        ##################################################
        self.uhd_usrp_sink_0 = uhd.usrp_sink(
            ",".join(("", "")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
            '',
        )
        self.uhd_usrp_sink_0.set_clock_source('external', 0)
        self.uhd_usrp_sink_0.set_center_freq(freq, 0)
        self.uhd_usrp_sink_0.set_gain(gain, 0)
        self.uhd_usrp_sink_0.set_samp_rate(samp_rate)
        self.uhd_usrp_sink_0.set_time_unknown_pps(uhd.time_spec())
        self.blocks_udp_source_0 = blocks.udp_source(gr.sizeof_gr_complex*1, '127.0.0.1', 8810, 1472, True)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_udp_source_0, 0), (self.uhd_usrp_sink_0, 0))

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
    parser = ArgumentParser(description=description)
    return parser


def main(top_block_cls=gr_cal_tcp_loopback_client, options=None):
    if options is None:
        options = argument_parser().parse_args()
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print("Error: failed to enable real-time scheduling.")
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.wait()


if __name__ == '__main__':
    main()
