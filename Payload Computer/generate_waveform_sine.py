#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Generate Waveform Sine
# Author: Krishna Makhija
# Description: GR flograph for generating a calibration waveform. The waveform will be ON/OFF keyed to mitigate multipath, enable phase consistency and averaging. This flowgraph will generate a pulsed sine wave with a rectangular envelope.
# Generated: Tue Jun  6 03:41:18 2023
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
import pmt
import sys
from gnuradio import qtgui


class generate_waveform_sine(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Generate Waveform Sine")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Generate Waveform Sine")
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

        self.settings = Qt.QSettings("GNU Radio", "generate_waveform_sine")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Variables
        ##################################################
        self.ON = ON = 4096
        self.samp_rate = samp_rate = 7.68e6
        self.OFF = OFF = 15*ON
        self.wave_samp_rate = wave_samp_rate = samp_rate
        self.wave_freq = wave_freq = samp_rate/8
        self.pulses = pulses = 96*2
        self.pulse_TX_time = pulse_TX_time = (ON+OFF)/samp_rate
        self.center_freq = center_freq = 150.96e6
        self.variable_tag_object_0 = variable_tag_object_0 = gr.tag_utils.python_to_tag((0, pmt.intern("key"), pmt.intern("value"), pmt.intern("src")))
        self.samps_per_period = samps_per_period = wave_samp_rate/wave_freq
        self.ring_buffer_size = ring_buffer_size = 4096
        self.pulse_ON_time = pulse_ON_time = ON/samp_rate
        self.pulse_OFF_time = pulse_OFF_time = OFF/samp_rate
        self.per_WP_time = per_WP_time = pulse_TX_time * pulses
        self.min_buffer = min_buffer = 512*4096
        self.head = head = (ON+OFF)
        self.freq = freq = center_freq - wave_freq

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
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/kmakhija/catkin_ws/src/Drone-Project-code/Payload Computer/sine_waveform', False)
        self.blocks_file_sink_1.set_unbuffered(False)
        self.analog_sig_source_x_0 = analog.sig_source_c(wave_samp_rate, analog.GR_COS_WAVE, wave_freq, 1, 0)



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
        self.settings = Qt.QSettings("GNU Radio", "generate_waveform_sine")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_ON(self):
        return self.ON

    def set_ON(self, ON):
        self.ON = ON
        self.set_head((self.ON+self.OFF))
        self.set_OFF(15*self.ON)
        self.set_pulse_TX_time((self.ON+self.OFF)/self.samp_rate)
        self.set_pulse_ON_time(self.ON/self.samp_rate)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_wave_samp_rate(self.samp_rate)
        self.set_wave_freq(self.samp_rate/8)
        self.set_pulse_TX_time((self.ON+self.OFF)/self.samp_rate)
        self.set_pulse_ON_time(self.ON/self.samp_rate)
        self.set_pulse_OFF_time(self.OFF/self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)

    def get_OFF(self):
        return self.OFF

    def set_OFF(self, OFF):
        self.OFF = OFF
        self.set_head((self.ON+self.OFF))
        self.set_pulse_TX_time((self.ON+self.OFF)/self.samp_rate)
        self.set_pulse_OFF_time(self.OFF/self.samp_rate)
        self.blocks_vector_source_x_0.set_data(np.hstack((np.zeros(self.OFF), np.ones(self.ON))), [])

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

    def get_pulses(self):
        return self.pulses

    def set_pulses(self, pulses):
        self.pulses = pulses
        self.set_per_WP_time(self.pulse_TX_time * self.pulses)

    def get_pulse_TX_time(self):
        return self.pulse_TX_time

    def set_pulse_TX_time(self, pulse_TX_time):
        self.pulse_TX_time = pulse_TX_time
        self.set_per_WP_time(self.pulse_TX_time * self.pulses)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq - self.wave_freq)

    def get_variable_tag_object_0(self):
        return self.variable_tag_object_0

    def set_variable_tag_object_0(self, variable_tag_object_0):
        self.variable_tag_object_0 = variable_tag_object_0

    def get_samps_per_period(self):
        return self.samps_per_period

    def set_samps_per_period(self, samps_per_period):
        self.samps_per_period = samps_per_period

    def get_ring_buffer_size(self):
        return self.ring_buffer_size

    def set_ring_buffer_size(self, ring_buffer_size):
        self.ring_buffer_size = ring_buffer_size

    def get_pulse_ON_time(self):
        return self.pulse_ON_time

    def set_pulse_ON_time(self, pulse_ON_time):
        self.pulse_ON_time = pulse_ON_time

    def get_pulse_OFF_time(self):
        return self.pulse_OFF_time

    def set_pulse_OFF_time(self, pulse_OFF_time):
        self.pulse_OFF_time = pulse_OFF_time

    def get_per_WP_time(self):
        return self.per_WP_time

    def set_per_WP_time(self, per_WP_time):
        self.per_WP_time = per_WP_time

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_head(self):
        return self.head

    def set_head(self, head):
        self.head = head
        self.blocks_head_0_0_0.set_length(self.head)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq


def main(top_block_cls=generate_waveform_sine, options=None):
    if gr.enable_realtime_scheduling() != gr.RT_OK:
        print "Error: failed to enable real-time scheduling."

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls()
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
