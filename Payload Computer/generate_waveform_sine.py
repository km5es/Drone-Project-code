#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Generate Waveform Sine
# Author: Krishna Makhija
# Description: GR flograph for generating a calibration waveform. The waveform will be ON/OFF keyed to mitigate multipath, enable phase consistency and averaging.
# Generated: Tue Jul 20 02:54:37 2021
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
from gnuradio import qtgui
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import numpy as np
import pmt
import sip
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
        self.samp_rate = samp_rate = 7.68e6/2
        self.wave_samp_rate = wave_samp_rate = samp_rate
        self.wave_freq = wave_freq = samp_rate/8
        self.ON_cycles = ON_cycles = 512
        self.samps_per_period = samps_per_period = wave_samp_rate/wave_freq
        self.OFF_cycles = OFF_cycles = 15*ON_cycles
        self.ON = ON = int(ON_cycles*samps_per_period)
        self.OFF = OFF = int(OFF_cycles*samps_per_period)
        self.stopTime = stopTime = ON/samp_rate
        self.head = head = ON+OFF
        self.center_freq = center_freq = 150e6
        self.t = t = np.linspace(-stopTime/2, stopTime/2, ON)
        self.freq = freq = center_freq - wave_freq
        self.Q = Q = head*4
        self.variable_tag_object_0 = variable_tag_object_0 = gr.tag_utils.python_to_tag((0, pmt.intern("key"), pmt.intern("value"), pmt.intern("src")))
        self.timeout = timeout = 4096/samp_rate
        self.ring_buffer_size = ring_buffer_size = 4096
        self.pulses = pulses = 96
        self.min_buffer = min_buffer = 512*4096
        self.gauss_envelope = gauss_envelope = np.exp(-(2*np.pi*freq*t/Q)**2)
        self.gain = gain = 20
        self.duty_cycle = duty_cycle = OFF/ON
        self.WAIT = WAIT = ON+OFF
        self.TX_time = TX_time = head/samp_rate
        self.PASS = PASS = ON+OFF
        self.ON_time = ON_time = ON_cycles/freq
        self.OFF_time = OFF_time = (OFF_cycles/freq)

        ##################################################
        # Blocks
        ##################################################
        self.qtgui_time_sink_x_0_0 = qtgui.time_sink_c(
        	1024*256, #size
        	samp_rate, #samp_rate
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_time_sink_x_0_0.set_update_time(0.10)
        self.qtgui_time_sink_x_0_0.set_y_axis(-1, 1)

        self.qtgui_time_sink_x_0_0.set_y_label('Amplitude', "")

        self.qtgui_time_sink_x_0_0.enable_tags(-1, True)
        self.qtgui_time_sink_x_0_0.set_trigger_mode(qtgui.TRIG_MODE_FREE, qtgui.TRIG_SLOPE_POS, 0.0, 0, 0, "")
        self.qtgui_time_sink_x_0_0.enable_autoscale(False)
        self.qtgui_time_sink_x_0_0.enable_grid(False)
        self.qtgui_time_sink_x_0_0.enable_axis_labels(True)
        self.qtgui_time_sink_x_0_0.enable_control_panel(True)
        self.qtgui_time_sink_x_0_0.enable_stem_plot(False)

        if not True:
          self.qtgui_time_sink_x_0_0.disable_legend()

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "blue"]
        styles = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        markers = [-1, -1, -1, -1, -1,
                   -1, -1, -1, -1, -1]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]

        for i in xrange(2):
            if len(labels[i]) == 0:
                if(i % 2 == 0):
                    self.qtgui_time_sink_x_0_0.set_line_label(i, "Re{{Data {0}}}".format(i/2))
                else:
                    self.qtgui_time_sink_x_0_0.set_line_label(i, "Im{{Data {0}}}".format(i/2))
            else:
                self.qtgui_time_sink_x_0_0.set_line_label(i, labels[i])
            self.qtgui_time_sink_x_0_0.set_line_width(i, widths[i])
            self.qtgui_time_sink_x_0_0.set_line_color(i, colors[i])
            self.qtgui_time_sink_x_0_0.set_line_style(i, styles[i])
            self.qtgui_time_sink_x_0_0.set_line_marker(i, markers[i])
            self.qtgui_time_sink_x_0_0.set_line_alpha(i, alphas[i])

        self._qtgui_time_sink_x_0_0_win = sip.wrapinstance(self.qtgui_time_sink_x_0_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_time_sink_x_0_0_win)
        self.qtgui_freq_sink_x_0 = qtgui.freq_sink_c(
        	4096, #size
        	firdes.WIN_BLACKMAN_hARRIS, #wintype
        	0, #fc
        	samp_rate, #bw
        	"", #name
        	1 #number of inputs
        )
        self.qtgui_freq_sink_x_0.set_update_time(0.10)
        self.qtgui_freq_sink_x_0.set_y_axis(-200, 10)
        self.qtgui_freq_sink_x_0.set_y_label('Relative Gain', 'dB')
        self.qtgui_freq_sink_x_0.set_trigger_mode(qtgui.TRIG_MODE_NORM, -18, 0, "")
        self.qtgui_freq_sink_x_0.enable_autoscale(False)
        self.qtgui_freq_sink_x_0.enable_grid(False)
        self.qtgui_freq_sink_x_0.set_fft_average(1.0)
        self.qtgui_freq_sink_x_0.enable_axis_labels(True)
        self.qtgui_freq_sink_x_0.enable_control_panel(True)

        if not True:
          self.qtgui_freq_sink_x_0.disable_legend()

        if "complex" == "float" or "complex" == "msg_float":
          self.qtgui_freq_sink_x_0.set_plot_pos_half(not True)

        labels = ['', '', '', '', '',
                  '', '', '', '', '']
        widths = [1, 1, 1, 1, 1,
                  1, 1, 1, 1, 1]
        colors = ["blue", "red", "green", "black", "cyan",
                  "magenta", "yellow", "dark red", "dark green", "dark blue"]
        alphas = [1.0, 1.0, 1.0, 1.0, 1.0,
                  1.0, 1.0, 1.0, 1.0, 1.0]
        for i in xrange(1):
            if len(labels[i]) == 0:
                self.qtgui_freq_sink_x_0.set_line_label(i, "Data {0}".format(i))
            else:
                self.qtgui_freq_sink_x_0.set_line_label(i, labels[i])
            self.qtgui_freq_sink_x_0.set_line_width(i, widths[i])
            self.qtgui_freq_sink_x_0.set_line_color(i, colors[i])
            self.qtgui_freq_sink_x_0.set_line_alpha(i, alphas[i])

        self._qtgui_freq_sink_x_0_win = sip.wrapinstance(self.qtgui_freq_sink_x_0.pyqwidget(), Qt.QWidget)
        self.top_grid_layout.addWidget(self._qtgui_freq_sink_x_0_win)
        self.blocks_vector_to_stream_0 = blocks.vector_to_stream(gr.sizeof_gr_complex*1, 4096)
        self.blocks_vector_source_x_0 = blocks.vector_source_c(np.hstack((np.zeros(OFF), np.ones(ON))), True, 1, [])
        (self.blocks_vector_source_x_0).set_min_output_buffer(4096)
        (self.blocks_vector_source_x_0).set_max_output_buffer(4096)
        self.blocks_throttle_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        (self.blocks_throttle_0).set_min_output_buffer(4096)
        (self.blocks_throttle_0).set_max_output_buffer(4096)
        self.blocks_stream_to_vector_0 = blocks.stream_to_vector(gr.sizeof_gr_complex*1, 4096)
        self.blocks_multiply_xx_0 = blocks.multiply_vcc(1)
        (self.blocks_multiply_xx_0).set_min_output_buffer(4096)
        (self.blocks_multiply_xx_0).set_max_output_buffer(4096)
        self.blocks_multiply_const_vxx_1 = blocks.multiply_const_vcc((gauss_envelope))
        (self.blocks_multiply_const_vxx_1).set_min_output_buffer(4096)
        (self.blocks_multiply_const_vxx_1).set_max_output_buffer(4096)
        self.blocks_multiply_const_vxx_0 = blocks.multiply_const_vcc((0.5, ))
        self.analog_sig_source_x_0 = analog.sig_source_c(wave_samp_rate, analog.GR_COS_WAVE, wave_freq, 1, 0)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_sig_source_x_0, 0), (self.blocks_multiply_xx_0, 0))
        self.connect((self.blocks_multiply_const_vxx_0, 0), (self.blocks_stream_to_vector_0, 0))
        self.connect((self.blocks_multiply_const_vxx_1, 0), (self.blocks_vector_to_stream_0, 0))
        self.connect((self.blocks_multiply_xx_0, 0), (self.blocks_throttle_0, 0))
        self.connect((self.blocks_stream_to_vector_0, 0), (self.blocks_multiply_const_vxx_1, 0))
        self.connect((self.blocks_throttle_0, 0), (self.blocks_multiply_const_vxx_0, 0))
        self.connect((self.blocks_vector_source_x_0, 0), (self.blocks_multiply_xx_0, 1))
        self.connect((self.blocks_vector_to_stream_0, 0), (self.qtgui_freq_sink_x_0, 0))
        self.connect((self.blocks_vector_to_stream_0, 0), (self.qtgui_time_sink_x_0_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "generate_waveform_sine")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.set_wave_samp_rate(self.samp_rate)
        self.set_wave_freq(self.samp_rate/8)
        self.set_timeout(4096/self.samp_rate)
        self.set_stopTime(self.ON/self.samp_rate)
        self.qtgui_time_sink_x_0_0.set_samp_rate(self.samp_rate)
        self.qtgui_freq_sink_x_0.set_frequency_range(0, self.samp_rate)
        self.blocks_throttle_0.set_sample_rate(self.samp_rate)
        self.set_TX_time(self.head/self.samp_rate)

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

    def get_ON(self):
        return self.ON

    def set_ON(self, ON):
        self.ON = ON
        self.set_t(np.linspace(-self.stopTime/2, self.stopTime/2, self.ON))
        self.set_stopTime(self.ON/self.samp_rate)
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

    def get_stopTime(self):
        return self.stopTime

    def set_stopTime(self, stopTime):
        self.stopTime = stopTime
        self.set_t(np.linspace(-self.stopTime/2, self.stopTime/2, self.ON))

    def get_head(self):
        return self.head

    def set_head(self, head):
        self.head = head
        self.set_TX_time(self.head/self.samp_rate)
        self.set_Q(self.head*4)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq - self.wave_freq)

    def get_t(self):
        return self.t

    def set_t(self, t):
        self.t = t
        self.set_gauss_envelope(np.exp(-(2*np.pi*self.freq*self.t/self.Q)**2))

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.set_gauss_envelope(np.exp(-(2*np.pi*self.freq*self.t/self.Q)**2))
        self.set_ON_time(self.ON_cycles/self.freq)
        self.set_OFF_time((self.OFF_cycles/self.freq))

    def get_Q(self):
        return self.Q

    def set_Q(self, Q):
        self.Q = Q
        self.set_gauss_envelope(np.exp(-(2*np.pi*self.freq*self.t/self.Q)**2))

    def get_variable_tag_object_0(self):
        return self.variable_tag_object_0

    def set_variable_tag_object_0(self, variable_tag_object_0):
        self.variable_tag_object_0 = variable_tag_object_0

    def get_timeout(self):
        return self.timeout

    def set_timeout(self, timeout):
        self.timeout = timeout

    def get_ring_buffer_size(self):
        return self.ring_buffer_size

    def set_ring_buffer_size(self, ring_buffer_size):
        self.ring_buffer_size = ring_buffer_size

    def get_pulses(self):
        return self.pulses

    def set_pulses(self, pulses):
        self.pulses = pulses

    def get_min_buffer(self):
        return self.min_buffer

    def set_min_buffer(self, min_buffer):
        self.min_buffer = min_buffer

    def get_gauss_envelope(self):
        return self.gauss_envelope

    def set_gauss_envelope(self, gauss_envelope):
        self.gauss_envelope = gauss_envelope
        self.blocks_multiply_const_vxx_1.set_k((self.gauss_envelope))

    def get_gain(self):
        return self.gain

    def set_gain(self, gain):
        self.gain = gain

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
