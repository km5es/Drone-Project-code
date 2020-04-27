### Paylod computer programs
These will go on the payload computer. Current candidates are a Raspberry Pi 4 and ODROID-XU4.
Both the gr_cal_tcp_loopback_client.py and cal_sequence_tcp_server.py files need to be added to cron jobs. The qpsk_waveform file needs to be in the same directory.
The cal_sequence_tcp_server.py file looks for a serial radio trigger from the base and begins writing qpsk_waveform to the GR file.
The GR file has multi_usrp block which transmits the calibration signal.
