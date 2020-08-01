#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Drone Pulse Tx Single Pol
# Author: Krishna Makhija
# Description: GR flograph for generating a calibration waveform. The waveform will be ON/OFF keyed to mitigate multipath, enable phase consistency and averaging.
# Generated: Fri Jul 31 20:49:15 2020
##################################################

if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"

from PyQt4 import Qt
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import numpy as np
import pmt
import sys
from gnuradio import qtgui


class drone_pulse_tx_single_pol(gr.top_block, Qt.QWidget):

    def __init__(self, device_add_1='serial=31993F7, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_add_2='serial=319940D, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_add_mini_0='serial=313B5A2, recv_frame_size=8200, num_recv_frames=512', device_gpsdo_1='serial=319098E, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_gpsdo_2='serial=31909C2, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex'):
        gr.top_block.__init__(self, "Drone Pulse Tx Single Pol")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Drone Pulse Tx Single Pol")
        qtgui.util.check_set_qss()
        try:
            self.setWindowIcon(Qt.QIcon.fromTheme('gnuradio-grc'))
        except:
            pass
        self.top_scroll_layout = Qt.QVBoxLayout()
        self.setLayout(self.top_scroll_layout)
        self.top_scroll = Qt.QScrollArea()
        self.top_scroll.setFrameStyle(Qt.QFrame.NoFrame)
        self.top_scroll_layout.addWidget(self.top_scroll)
        self.top_scroll.setWidgetResizable(True)
        self.top_widget = Qt.QWidget()
        self.top_scroll.setWidget(self.top_widget)
        self.top_layout = Qt.QVBoxLayout(self.top_widget)
        self.top_grid_layout = Qt.QGridLayout()
        self.top_layout.addLayout(self.top_grid_layout)

        self.settings = Qt.QSettings("GNU Radio", "drone_pulse_tx_single_pol")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Parameters
        ##################################################
        self.device_add_1 = device_add_1
        self.device_add_2 = device_add_2
        self.device_add_mini_0 = device_add_mini_0
        self.device_gpsdo_1 = device_gpsdo_1
        self.device_gpsdo_2 = device_gpsdo_2

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 7.68e6*2
        self.wave_samp_rate = wave_samp_rate = samp_rate
        self.wave_freq = wave_freq = samp_rate/8
        self.ON_cycles = ON_cycles = 512
        self.samps_per_period = samps_per_period = wave_samp_rate/wave_freq
        self.OFF_cycles = OFF_cycles = 15*ON_cycles
        self.center_freq = center_freq = 150e6
        self.ON = ON = int(ON_cycles*samps_per_period)
        self.OFF = OFF = int(OFF_cycles*samps_per_period)
        self.head = head = ON+OFF
        self.freq = freq = center_freq - wave_freq
        self.toggle_start = toggle_start = 0
        self.timeout = timeout = 4096/samp_rate
        self.sps = sps = 4
        self.ring_buffer_size = ring_buffer_size = 4096
        self.qpsk = qpsk = digital.constellation_rect(([0.707+0.707j, -0.707+0.707j, -0.707-0.707j, 0.707-0.707j]), ([0, 1, 2, 3]), 4, 2, 2, 1, 1).base()
        self.pulses = pulses = 96
        self.min_buffer = min_buffer = 512*4096
        self.head_0 = head_0 = 512*8192
        self.gain = gain = 20
        self.excess_bw = excess_bw = 0.35
        self.duty_cycle = duty_cycle = OFF/ON
        self.WAIT = WAIT = ON+OFF
        self.TX_time = TX_time = head/samp_rate
        self.PASS = PASS = ON+OFF
        self.ON_time = ON_time = ON_cycles/freq
        self.OFF_time = OFF_time = (OFF_cycles/freq)

        ##################################################
        # Blocks
        ##################################################
        self.digital_constellation_modulator_0 = digital.generic_mod(
          constellation=qpsk,
          differential=True,
          samples_per_symbol=sps,
          pre_diff_code=True,
          excess_bw=excess_bw,
          verbose=False,
          log=False,
          )
        (self.digital_constellation_modulator_0).set_min_output_buffer(4096)
        (self.digital_constellation_modulator_0).set_max_output_buffer(4096)
        self.blocks_vector_source_x_0 = blocks.vector_source_c(np.hstack((np.zeros(OFF), np.ones(ON))), True, 1, [])
        (self.blocks_vector_source_x_0).set_min_output_buffer(4096)
        (self.blocks_vector_source_x_0).set_max_output_buffer(4096)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        (self.blocks_throttle_0).set_min_output_buffer(4096)
        (self.blocks_throttle_0).set_max_output_buffer(4096)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        (self.blocks_multiply_xx_0).set_min_output_buffer(4096)
        (self.blocks_multiply_xx_0).set_max_output_buffer(4096)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((0.5, ))
        self.blocks_head_0_0_0 = blocks.head(gr.sizeof_gr_complex*1, head)
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_char*1, '/home/kmakhija/Drone-Project/Payload Computer/keyword.dat', True)
        self.blocks_file_source_0.set_begin_tag(pmt.PMT_NIL)
        (self.blocks_file_source_0).set_min_output_buffer(4096)
        (self.blocks_file_source_0).set_max_output_buffer(4096)
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/kmakhija/Drone-Project/Payload Computer/qpsk_waveform', False)
        self.blocks_file_sink_1.set_unbuffered(False)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_file_source_0, 0), (self.digital_constellation_modulator_0, 0))
        self.connect((self.blocks_head_0_0_0, 0), (self.blocks_file_sink_1, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_head_0_0_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.digital_constellation_modulator_0, 0), (self.blocks_multiply_xx_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "drone_pulse_tx_single_pol")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_device_add_1(self):
        return self.device_add_1

    def set_device_add_1(self, device_add_1):
        self.device_add_1 = device_add_1

    def get_device_add_2(self):
        return self.device_add_2

    def set_device_add_2(self, device_add_2):
        self.device_add_2 = device_add_2

    def get_device_add_mini_0(self):
        return self.device_add_mini_0

    def set_device_add_mini_0(self, device_add_mini_0):
        self.device_add_mini_0 = device_add_mini_0

    def get_device_gpsdo_1(self):
        return self.device_gpsdo_1

    def set_device_gpsdo_1(self, device_gpsdo_1):
        self.device_gpsdo_1 = device_gpsdo_1

    def get_device_gpsdo_2(self):
        return self.device_gpsdo_2

    def set_device_gpsdo_2(self, device_gpsdo_2):
        self.device_gpsdo_2 = device_gpsdo_2

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_wave_samp_rate(self.samp_rate)
        self.set_wave_freq(self.samp_rate/8)
        self.set_timeout(4096/self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)
        self.set_TX_time(self.head/self.samp_rate)

    def get_wave_samp_rate(self):
        return self.wave_samp_rate

    def set_wave_samp_rate(self, wave_samp_rate):
        self.wave_samp_rate = wave_samp_rate
        self.set_samps_per_period(self.wave_samp_rate/self.wave_freq)

    def get_wave_freq(self):
        return self.wave_freq

    def set_wave_freq(self, wave_freq):
        self.wave_freq = wave_freq
        self.set_samps_per_period(self.wave_samp_rate/self.wave_freq)
        self.set_freq(self.center_freq - self.wave_freq)

    def get_ON_cycles(self):
        return self.ON_cycles

    def set_ON_cycles(self, ON_cycles):
        self.ON_cycles = ON_cycles
        self.set_ON(int(self.ON_cycles*self.samps_per_period))
        self.set_ON_time(self.ON_cycles/self.freq)
        self.set_OFF_cycles(15*self.ON_cycles)

    def get_samps_per_period(self):
        return self.samps_per_period

    def set_samps_per_period(self, samps_per_period):
        self.samps_per_period = samps_per_period
        self.set_ON(int(self.ON_cycles*self.samps_per_period))
        self.set_OFF(int(self.OFF_cycles*self.samps_per_period))

    def get_OFF_cycles(self):
        return self.OFF_cycles

    def set_OFF_cycles(self, OFF_cycles):
        self.OFF_cycles = OFF_cycles
        self.set_OFF(int(self.OFF_cycles*self.samps_per_period))
        self.set_OFF_time((self.OFF_cycles/self.freq))

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq - self.wave_freq)

    def get_ON(self):
        return self.ON

    def set_ON(self, ON):
        self.ON = ON
        self.set_head(self.ON+self.OFF)
        self.set_duty_cycle(self.OFF/self.ON)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])
        self.set_WAIT(self.ON+self.OFF)
        self.set_PASS(self.ON+self.OFF)

    def get_OFF(self):
        return self.OFF

    def set_OFF(self, OFF):
        self.OFF = OFF
        self.set_head(self.ON+self.OFF)
        self.set_duty_cycle(self.OFF/self.ON)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])
        self.set_WAIT(self.ON+self.OFF)
        self.set_PASS(self.ON+self.OFF)

    def get_head(self):
        return self.head

    def set_head(self, head):
        self.head = head
        self.blocks_head_0_0_0.set_length(self.head)
        self.set_TX_time(self.head/self.samp_rate)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_ON_time(self.ON_cycles/self.freq)
        self.set_OFF_time((self.OFF_cycles/self.freq))

    def get_toggle_start(self):
        return self.toggle_start

    def set_toggle_start(self, toggle_start):
        self.toggle_start = toggle_start

    def get_timeout(self):
        return self.timeout

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps

    def get_ring_buffer_size(self):
        return self.ring_buffer_size

    def set_ring_buffer_size(self, ring_buffer_size):
        self.ring_buffer_size = ring_buffer_size

    def get_qpsk(self):
        return self.qpsk

    def set_qpsk(self, qpsk):
        self.qpsk = qpsk

    def get_pulses(self):
        return self.pulses

    def set_pulses(self, pulses):
        self.pulses = pulses

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_head_0(self):
        return self.head_0

    def set_head_0(self, head_0):
        self.head_0 = head_0

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain

    def get_excess_bw(self):
        return self.excess_bw

    def set_excess_bw(self, excess_bw):
        self.excess_bw = excess_bw

    def get_duty_cycle(self):
        return self.duty_cycle

    def set_duty_cycle(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def get_WAIT(self):
        return self.WAIT

    def set_WAIT(self, WAIT):
        self.WAIT = WAIT

    def get_TX_time(self):
        return self.TX_time

    def set_TX_time(self, TX_time):
        self.TX_time = TX_time

    def get_PASS(self):
        return self.PASS

    def set_PASS(self, PASS):
        self.PASS = PASS

    def get_ON_time(self):
        return self.ON_time

    def set_ON_time(self, ON_time):
        self.ON_time = ON_time

    def get_OFF_time(self):
        return self.OFF_time

    def set_OFF_time(self, OFF_time):
        self.OFF_time = OFF_time


def argument_parser():
    description = 'GR flograph for generating a calibration waveform. The waveform will be ON/OFF keyed to mitigate multipath, enable phase consistency and averaging.'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--device-add-1", dest="device_add_1", type="string", default='serial=31993F7, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex',
        help="Set device_add_1 [default=%default]")
    parser.add_option(
        "", "--device-add-2", dest="device_add_2", type="string", default='serial=319940D, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex',
        help="Set device_add_2 [default=%default]")
    parser.add_option(
        "", "--device-add-mini-0", dest="device_add_mini_0", type="string", default='serial=313B5A2, recv_frame_size=8200, num_recv_frames=512',
        help="Set B205mini [default=%default]")
    parser.add_option(
        "", "--device-gpsdo-1", dest="device_gpsdo_1", type="string", default='serial=319098E, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex',
        help="Set device_gpsdo_1 [default=%default]")
    parser.add_option(
        "", "--device-gpsdo-2", dest="device_gpsdo_2", type="string", default='serial=31909C2, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex',
        help="Set device_gpsdo_2 [default=%default]")
    return parser


def main(top_block_cls=drone_pulse_tx_single_pol, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(device_add_1=options.device_add_1, device_add_2=options.device_add_2, device_add_mini_0=options.device_add_mini_0, device_gpsdo_1=options.device_gpsdo_1, device_gpsdo_2=options.device_gpsdo_2)
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
