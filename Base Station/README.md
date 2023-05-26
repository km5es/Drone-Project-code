# Base station 🖥️

>**NOTE**: The current build only works with Ubuntu 18.04, ROS Melodic, UHD 3.15.0.0, and GNU Radio 3.7.13.4. All of these work on Python 2.7.

The base station consists of:
- a laptop, 
- a B210 SDR, 
- 910 MHz telemetry radio, and
- the RTK GPS base. 

The laptop will require a reliable and dedicated USB 3.0 controller for high-speed data acquisition from the SDR. Default sampling rates are 7.68 MSPS dual-channel which translates to 983.04 Mbps of data throughput. This also requires a fast hard drive, preferably an NVMe SSD but a SATA SSD should also work. The code may also be modified to hold the data in RAM, but that would require >32 GB of RAM for flight sorties exceeding about 3 minutes or so. 

## Data acquisition during field tests 💽
During beam mapping on the field, there are two ways of acquiring data. 

1. The first method will continuously save SDR data and temperature information from the moment it is instantiated. This script is headless and does not display received signal or any other diagnostic information. To initiate this, open up a console and:
```
roscd beam_mapping/Base\ Station/
./begin_mission.sh
```
This will open up a new terminal window in which a ROS node is continuously saving temperature data to a log file located in the [`./logs/base/`](/logs/base). The original terminal runs the SDR RF data acquisition. To stop acquisition, simply close the new terminals.

2. The other method will have a GUI which displays received signal and has some terminal commands to begin a synchronized calibration with the payload over a UDP connection. The terminal commands also allow one to begin and stop data acquisition using custom keyboard shortcuts (`Ctrl+Alt+Q` to begin and `Ctrl+Alt+P` to stop). 
```
roscd beam_mapping/Base\ Station/
python tcp_toggle.py
```
In another terminal:
```
roscd beam_mapping/Base\ Station/
python base-station-receiver.py
```
This method is more convenient for field measurements since one has visual confirmation of the measurements in real-time. However, the caveat is that it does not record SDR temperature changes for post-processing. 