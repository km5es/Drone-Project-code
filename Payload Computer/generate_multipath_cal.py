#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Generate Multipath Cal
# Author: Krishna Makhija
# Description: GR flograph for generating a multipath calibration signals.
# Generated: Tue Sep 14 02:55:00 2021
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
from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import numpy as np
import sys
from gnuradio import qtgui


class generate_multipath_cal(gr.top_block, Qt.QWidget):

    def __init__(self, device_add_1='serial=31993F7, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_add_2='serial=319940D, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_add_mini_0='serial=313B5A2, recv_frame_size=8200, num_recv_frames=512', device_gpsdo_1='serial=319098E, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex', device_gpsdo_2='serial=31909C2, recv_frame_size=8200, num_recv_frames=512, fpga=usrp_b210_fpga.bin, fw=usrp_b200_fw.hex'):
        gr.top_block.__init__(self, "Generate Multipath Cal")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Generate Multipath Cal")
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

        self.settings = Qt.QSettings("GNU Radio", "generate_multipath_cal")
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
        self.samp_rate = samp_rate = 7.5e6
        self.wave_samp_rate = wave_samp_rate = samp_rate
        self.wave_freq = wave_freq = samp_rate/3
        self.ON_cycles = ON_cycles = 2
        self.samps_per_period = samps_per_period = wave_samp_rate/wave_freq
        self.OFF_cycles = OFF_cycles = 15*ON_cycles
        self.pulses = pulses = 96
        self.head = head = 72000
        self.ON = ON = int(ON_cycles*samps_per_period)
        self.OFF = OFF = int(OFF_cycles*samps_per_period)
        self.total_pulses = total_pulses = pulses * (head/(ON+OFF))
        self.pulse_time = pulse_time = (ON+OFF)/samp_rate
        self.center_freq = center_freq = 150e6
        self.ring_buffer_size = ring_buffer_size = 4096
        self.min_buffer = min_buffer = 512*4096
        self.gain = gain = 20
        self.freq = freq = center_freq - wave_freq
        self.duty_cycle = duty_cycle = OFF/ON
        self.TX_time = TX_time = pulse_time * total_pulses
        self.ON_time = ON_time = ON/samp_rate
        self.OFF_time = OFF_time = OFF/samp_rate

        ##################################################
        # Blocks
        ##################################################
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
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/kmakhija/catkin_ws/src/Drone-Project-code/Payload Computer/multipath_cal', False)
        self.blocks_file_sink_1.set_unbuffered(False)
        self.analog_sig_source_x_0 = analog.sig_source_c(wave_samp_rate, analog.GR_COS_WAVE, wave_freq, 0.25, 0)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.blocks_head_0_0_0, 0), (self.blocks_file_sink_1, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_head_0_0_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_multiply_xx_0, 1))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "generate_multipath_cal")
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
        self.set_wave_freq(self.samp_rate/3)
        self.set_pulse_time((self.ON+self.OFF)/self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)
        self.set_ON_time(self.ON/self.samp_rate)
        self.set_OFF_time(self.OFF/self.samp_rate)

    def get_wave_samp_rate(self):
        return self.wave_samp_rate

    def set_wave_samp_rate(self, wave_samp_rate):
        self.wave_samp_rate = wave_samp_rate
        self.set_samps_per_period(self.wave_samp_rate/self.wave_freq)
        self.analog_sig_source_x_0.set_sampling_freq(self.wave_samp_rate)

    def get_wave_freq(self):
        return self.wave_freq

    def set_wave_freq(self, wave_freq):
        self.wave_freq = wave_freq
        self.set_samps_per_period(self.wave_samp_rate/self.wave_freq)
        self.set_freq(self.center_freq - self.wave_freq)
        self.analog_sig_source_x_0.set_frequency(self.wave_freq)

    def get_ON_cycles(self):
        return self.ON_cycles

    def set_ON_cycles(self, ON_cycles):
        self.ON_cycles = ON_cycles
        self.set_ON(int(self.ON_cycles*self.samps_per_period))
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

    def get_pulses(self):
        return self.pulses

    def set_pulses(self, pulses):
        self.pulses = pulses
        self.set_total_pulses(self.pulses * (self.head/(self.ON+self.OFF)))

    def get_head(self):
        return self.head

    def set_head(self, head):
        self.head = head
        self.set_total_pulses(self.pulses * (self.head/(self.ON+self.OFF)))
        self.blocks_head_0_0_0.set_length(self.head)

    def get_ON(self):
        return self.ON

    def set_ON(self, ON):
        self.ON = ON
        self.set_total_pulses(self.pulses * (self.head/(self.ON+self.OFF)))
        self.set_pulse_time((self.ON+self.OFF)/self.samp_rate)
        self.set_duty_cycle(self.OFF/self.ON)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])
        self.set_ON_time(self.ON/self.samp_rate)

    def get_OFF(self):
        return self.OFF

    def set_OFF(self, OFF):
        self.OFF = OFF
        self.set_total_pulses(self.pulses * (self.head/(self.ON+self.OFF)))
        self.set_pulse_time((self.ON+self.OFF)/self.samp_rate)
        self.set_duty_cycle(self.OFF/self.ON)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])
        self.set_OFF_time(self.OFF/self.samp_rate)

    def get_total_pulses(self):
        return self.total_pulses

    def set_total_pulses(self, total_pulses):
        self.total_pulses = total_pulses
        self.set_TX_time(self.pulse_time * self.total_pulses)

    def get_pulse_time(self):
        return self.pulse_time

    def set_pulse_time(self, pulse_time):
        self.pulse_time = pulse_time
        self.set_TX_time(self.pulse_time * self.total_pulses)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq - self.wave_freq)

    def get_ring_buffer_size(self):
        return self.ring_buffer_size

    def set_ring_buffer_size(self, ring_buffer_size):
        self.ring_buffer_size = ring_buffer_size

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq

    def get_duty_cycle(self):
        return self.duty_cycle

    def set_duty_cycle(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def get_TX_time(self):
        return self.TX_time

    def set_TX_time(self, TX_time):
        self.TX_time = TX_time

    def get_ON_time(self):
        return self.ON_time

    def set_ON_time(self, ON_time):
        self.ON_time = ON_time

    def get_OFF_time(self):
        return self.OFF_time

    def set_OFF_time(self, OFF_time):
        self.OFF_time = OFF_time


def argument_parser():
    description = 'GR flograph for generating a multipath calibration signals.'
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


def main(top_block_cls=generate_multipath_cal, options=None):
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
