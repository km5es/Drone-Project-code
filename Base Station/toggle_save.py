#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Save Data
# Author: Krishna Makhija
# Description: This flowgraph will simply save data when the checkbox is ticked.
# Generated: Wed Jul 21 19:38:17 2021
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
from gnuradio import uhd
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from grc_gnuradio import blks2 as grc_blks2
from optparse import OptionParser
import sys
import time
from gnuradio import qtgui


class save_data(gr.top_block, Qt.QWidget):

    def __init__(self, timestamp=time.strftime("%H%M%S-%d%m%Y")):
        gr.top_block.__init__(self, "Save Data")
        Qt.QWidget.__init__(self)
        self.setWindowTitle("Save Data")
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

        self.settings = Qt.QSettings("GNU Radio", "save_data")
        self.restoreGeometry(self.settings.value("geometry").toByteArray())


        ##################################################
        # Parameters
        ##################################################
        self.timestamp = timestamp

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 3.84e6
        self.wave_freq = wave_freq = samp_rate/8
        self.meas_freq = meas_freq = 150e6
        self.toggle = toggle = 0
        self.min_buffer = min_buffer = 512*8200*2
        self.gain = gain = 60
        self.freq = freq = meas_freq - wave_freq

        ##################################################
        # Blocks
        ##################################################
        _toggle_check_box = Qt.QCheckBox("toggle")
        self._toggle_choices = {True: 1, False: 0}
        self._toggle_choices_inv = dict((v,k) for k,v in self._toggle_choices.iteritems())
        self._toggle_callback = lambda i: Qt.QMetaObject.invokeMethod(_toggle_check_box, "setChecked", Qt.Q_ARG("bool", self._toggle_choices_inv[i]))
        self._toggle_callback(self.toggle)
        _toggle_check_box.stateChanged.connect(lambda i: self.set_toggle(self._toggle_choices[bool(i)]))
        self.top_grid_layout.addWidget(_toggle_check_box)
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
        self.uhd_usrp_source_0.set_gain(gain, 0)
        (self.uhd_usrp_source_0).set_min_output_buffer(8396800)
        self.blocks_null_sink_0_0 = blocks.null_sink(gr.sizeof_gr_complex*1)
        self.blocks_file_sink_0 = blocks.file_sink(gr.sizeof_gr_complex*1, '/mnt/78ACE633ACE5EB96/milton_raw_data/' +str(self.timestamp) + 'cal_data.dat', True)
        self.blocks_file_sink_0.set_unbuffered(False)
        self.blks2_selector_0_0_0 = grc_blks2.selector(
        	item_size=gr.sizeof_gr_complex*1,
        	num_inputs=1,
        	num_outputs=2,
        	input_index=0,
        	output_index=toggle,
        )



        ##################################################
        # Connections
        ##################################################
        self.connect((self.blks2_selector_0_0_0, 1), (self.blocks_file_sink_0, 0))
        self.connect((self.blks2_selector_0_0_0, 0), (self.blocks_null_sink_0_0, 0))
        self.connect((self.uhd_usrp_source_0, 0), (self.blks2_selector_0_0_0, 0))

    def closeEvent(self, event):
        self.settings = Qt.QSettings("GNU Radio", "save_data")
        self.settings.setValue("geometry", self.saveGeometry())
        event.accept()

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

    def get_toggle(self):
        return self.toggle

    def set_toggle(self, toggle):
        self.toggle = toggle
        self._toggle_callback(self.toggle)
        self.blks2_selector_0_0_0.set_output_index(int(self.toggle))

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


def argument_parser():
    description = 'This flowgraph will simply save data when the checkbox is ticked.'
    parser = OptionParser(usage="%prog: [options]", option_class=eng_option, description=description)
    parser.add_option(
        "", "--timestamp", dest="timestamp", type="string", default=time.strftime("%H%M%S-%d%m%Y"),
        help="Set timestamp [default=%default]")
    return parser


def main(top_block_cls=save_data, options=None):
    if options is None:
        options, _ = argument_parser().parse_args()

    from distutils.version import StrictVersion
    if StrictVersion(Qt.qVersion()) >= StrictVersion("4.5.0"):
        style = gr.prefs().get_string('qtgui', 'style', 'raster')
        Qt.QApplication.setGraphicsSystem(style)
    qapp = Qt.QApplication(sys.argv)

    tb = top_block_cls(timestamp=options.timestamp)
    tb.start()
    tb.show()

    def quitting():
        tb.stop()
        tb.wait()
    qapp.connect(qapp, Qt.SIGNAL("aboutToQuit()"), quitting)
    qapp.exec_()


if __name__ == '__main__':
    main()
