## Payload computer programs
This folder contains all the codes needed to run the calibration signal using the single board computer on the drone payload.
* [This](drone_pulse_tx_single_pol_save_file.grc) GNU Radio flowgraph is used for generating a pre-stored waveform.
* [This](generate_waveform_qpsk.grc) is explicitly designed to generate a QPSK-modulated waveform and [this](generate_waveform_sine) is used for explicitly generating an amplitude modulated sine waveform.
* Both the TCP [client](gr_cal_tcp_loopback_client.py) and [server](cal_sequence_tcp_server.py) files need to be added to cron jobs on the payload computer. The [qpsk_waveform](qpsk_Waveform) file needs to be in the same (working) directory. So does the [zeros](zeros) file.
* A shell script, [start_cal_v2.sh](start_cal_v2.sh) runs both at startup and saves a log of all output to the home directory.

Current candidates for the payload computer are: [Raspberry Pi 4B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/), [NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2) with the [Orbitty carrier board](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/), and an [ODROID XU4](https://www.hardkernel.com/shop/odroid-xu4-special-price/). I am currently moving ahead with a Raspberry Pi 4B with Ubuntu server 18.04.

Here's how to set things up on the payload computer. 

### Dependencies
Some dependencies are not listed on the Ettus website. I kept getting cmake errors on the Raspberry Pi 4B when trying to build from the UHd repo. Start by doing this:
```
sudo apt install python-pip python3-pip aptitude
pip install serial pyserial termcolor numpy scipy mako
```
Then install GNU Radio and UHD by following instructions from the main README file.

### ROS
The Raspbery Pi version of ROS can be downloaded by following instructions [here][ROSberryPi_link] when using Raspbian. For a vanilla Ubuntu 18.04 or Mate 18.04 use [this][melodic_install] instead. 

For acquiring metadata the flight controller needs to be connected to the payload computer using a USB interface.
For the ROS topics to be heard by the payload computer, a MAVROS node needs to be running. Install MAVROS package:
```
sudo apt install ros-melodic-mavros
```

### Clone the repo on the payload computer:
```
cd /home/$USER/
git clone https://github.com/km5es/Drone-Project-code.git
```
#### Set up payload for running the ROS + SDR code
The payload computer will be interfaced via USB to the FC, SDR, and telemetry radios. The Pixhawk 3 Pro is set to have its TELEM1, TELEM2, and TELEM3/4 ports to transmit MAVLink compatible messages. Relevant FC params:
```
MAV_2_CONFIG = TELEM3/4
SER_TEL2_BAUD = 921600
```
The FC is interfaced to the Pi using a custom cable and a FTDI to JST-GH [adapter][]. Since there could be conflicting USB port allocations at boot-up one needs to add symlinks to each device. This is done by plugging in each device into a USB port and:
```
udevadm info /dev/ttyUSB0 | grep 'VENDOR_ID\|ID_MODEL_ID\|ID_SERIAL_SHORT'
```
Here is an example output when the FC is plugged in:
```
E: ID_MODEL_ID=6015
E: ID_SERIAL_SHORT=DM01KU0E
E: ID_VENDOR_ID=0403
```
Enter those data in `/etc/udev/rules.d/99-pixhawk.rules` like so:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DM01KU0E", SYMLINK+="ttyFC"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="D308LPAN", SYMLINK+="ttyTELEM"
```
Reboot for it to take effect. 
>**Note:** The scripts here use the symlinks mentioned above. Please be sure to either change that in the code, or use the exact same symlinks.

Finally, add a cron job to run these scripts at startup.
```
crontab -e
```
And then add the following line to the end of the file:
```
@reboot bash -c 'source /opt/ros/melodic/setup.bash; ~/Drone-Project-code/Payload\ Computer/connect_to_fc.sh' >~/Drone-Project-code/logs/fc_connect 2>&1
@reboot bash -c 'source /opt/ros/melodic/setup.bash; ~/Drone-Project-code/Payload\ Computer/start_cal_v2.sh' >~/Drone-Project-code/logs/cal_log 2>&1
```
To recover logs of the stdout from the above cron jobs:
```
cat ~/Drone-Project-code/logs/fc_connect
cat ~/Drone-Project-code/logs/cal_log
```
>**Note:** The stdout of the ROS and SDR codes are in fc_connect and cal_log. However, detailed and timestamped event logs are saved in the `./logs` folder within the git repo.


[ROSberryPi_link]: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi
[melodic_install]: http://wiki.ros.org/melodic/Installation/Ubuntu
[adapter]: https://store.mrobotics.io/USB-FTDI-Serial-to-JST-GH-p/mro-ftdi-jstgh01-mr.htm

@reboot bash -c 'source /opt/ros/kinetic/setup.bash; ~/Drone-Project-code/Payload\ Computer/connect_to_fc.sh' >~/Drone-Project-code/logs/fc_connect 2>&1
@reboot bash -c 'source /opt/ros/kinetic/setup.bash; ~/Drone-Project-code/Payload\ Computer/start_cal_v2.sh' >~/Drone-Project-code/logs/cal_log 2>&1
