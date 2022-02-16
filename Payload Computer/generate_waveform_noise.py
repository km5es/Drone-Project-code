#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Generate Waveform Noise
# Generated: Tue Feb 15 18:38:50 2022
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
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import numpy as np
import pmt
import sys
from gnuradio import qtgui


class generate_waveform_noise(gr.top_block, Qt.QWidget):

    def __init__(self):
        gr.top_block.__init__(self, "Generate Waveform Noise")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Generate Waveform Noise")
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

        self.settings = Qt.QSettings("GNU Radio", "generate_waveform_noise")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Variables
        ##################################################
        self.ON = ON = 4096
        self.samp_rate = samp_rate = 7.68e6
        self.OFF = OFF = 15*ON
        self.ring_buffer_size = ring_buffer_size = 4096
        self.head = head = 65536
        self.duty_cycle = duty_cycle = OFF/ON
        self.ON_time = ON_time = ON/samp_rate
        self.OFF_time = OFF_time = OFF/samp_rate

        ##################################################
        # Blocks
        ##################################################
        self.blocks_throttle_0_0 = blocks.throttle(gr.sizeof_gr_complex*1, samp_rate,True)
        self.blocks_head_0_0 = blocks.head(gr.sizeof_gr_complex*1, head*32)
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_gr_complex*1, '/home/kmakhija/catkin_ws/src/Drone-Project-code/Payload Computer/noise', True)
        self.blocks_file_source_0.set_begin_tag(pmt.PMT_NIL)
        self.blocks_file_sink_1 = blocks.file_sink(gr.sizeof_gr_complex*1, '/home/kmakhija/catkin_ws/src/Drone-Project-code/Payload Computer/noise', False)
        self.blocks_file_sink_1.set_unbuffered(False)



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_file_source_0, 0), (self.blocks_throttle_0_0, 0))
        self.connect((self.blocks_head_0_0, 0), (self.blocks_file_sink_1, 0))
        self.connect((self.blocks_throttle_0_0, 0), (self.blocks_head_0_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "generate_waveform_noise")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

    def get_ON(self):
        return self.ON

    def set_ON(self, ON):
        self.ON = ON
        self.set_duty_cycle(self.OFF/self.ON)
        self.set_ON_time(self.ON/self.samp_rate)
        self.set_OFF(15*self.ON)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.blocks_throttle_0_0.set_sample_rate(self.samp_rate)
        self.set_ON_time(self.ON/self.samp_rate)
        self.set_OFF_time(self.OFF/self.samp_rate)

    def get_OFF(self):
        return self.OFF

    def set_OFF(self, OFF):
        self.OFF = OFF
        self.set_duty_cycle(self.OFF/self.ON)
        self.set_OFF_time(self.OFF/self.samp_rate)

    def get_ring_buffer_size(self):
        return self.ring_buffer_size

    def set_ring_buffer_size(self, ring_buffer_size):
        self.ring_buffer_size = ring_buffer_size

    def get_head(self):
        return self.head

    def set_head(self, head):
        self.head = head
        self.blocks_head_0_0.set_length(self.head*32)

    def get_duty_cycle(self):
        return self.duty_cycle

    def set_duty_cycle(self, duty_cycle):
        self.duty_cycle = duty_cycle

    def get_ON_time(self):
        return self.ON_time

    def set_ON_time(self, ON_time):
        self.ON_time = ON_time

    def get_OFF_time(self):
        return self.OFF_time

    def set_OFF_time(self, OFF_time):
        self.OFF_time = OFF_time


def main(top_block_cls=generate_waveform_noise, options=None):

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
