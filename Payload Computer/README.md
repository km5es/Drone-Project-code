## Payload computer programs
This folder contains all the codes needed to run the calibration signal using the single board computer on the drone payload.
* [This](drone_pulse_tx_single_pol_save_file.grc) GNU Radio flowgraph is used for generating a pre-stored waveform.
* [This](generate_waveform_qpsk.grc) is explicitly designed to generate a QPSK-modulated waveform and [this](generate_waveform_sine) is used for explicitly generating an amplitude modulated sine waveform.
* Both the TCP [client](gr_cal_tcp_loopback_client.py) and [server](cal_sequence_tcp_server.py) files need to be added to cron jobs on the payload computer. The [qpsk_waveform](qpsk_Waveform) file needs to be in the same (working) directory. So does the [zeros](zeros) file.
* A shell script, [start_cal_v2.sh](start_cal_v2.sh) runs both at startup and saves a log of all output to the home directory.

Current candidates for the payload computer are: [Raspberry Pi 4B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/), [NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2) with the [Orbitty carrier board](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/), and an [ODROID XU4](https://www.hardkernel.com/shop/odroid-xu4-special-price/).

Here's how to set things up on the payload computer. 

### Dependencies
Currently testing everything using Ubuntu 20.04, ROS Noetic, Python 3.0, GNU Radio 3.8, and UHD 3.15. 

#### SDR-related dependencies
Instructions [here][uhd] and [here][uhd2].
```
sudo apt-get install libboost-all-dev libusb-1.0-0-dev python-mako doxygen python-docutils cmake build-essential
sudo apt-get install libuhd-dev libuhd003 uhd-host
```
>****Note:**** If libuhd003 is not found, skip installing it. This will still give you the latest version of UHD and GNU Radio and it seems to work fine for me now.


#### ROS
The Raspbery Pi version of ROS can be downloaded by following instructions [here][ROSberryPi_link] when using Raspbian. For a vanilla Ubuntu 18.04 or Mate 18.04 use [this][noetic_install] instead. 
Add the following to the ~/.bashrc file:
```
source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export PYTHONPATH=/usr/lib/python3/dist-packages:/usr/lib/python3/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=/usr/lib:$LD_LIBRARY_PATH
```

Then set up a catkin workspace:
```
mkdir ~/catkin_ws/src/ -p && cd ~/catkin_ws/
catkin_make
```


#### Additional dependencies
```
sudo apt install python3-pip aptitude git
pip3 install serial pyserial termcolor numpy scipy mako
sudo apt install ros-noetic-mavros
```


### Clone the repo on the payload computer:
```
cd /home/$USER/
git clone https://github.com/km5es/Drone-Project-code.git
```

#### Configure USB rules:
USB udev rules need to be configured using instructions found [here][uhd_install_from_git]. For convenience, the instructions are reproduced here as well.
```
cd /home/$USER/Drone-Project-code/Payload\ Computer
sudo cp uhd-usrp.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo groupadd usrp
sudo usermod -aG usrp $USER
```
Then add the following line to `/etc/security/limits.conf`
```
@usrp - rtprio  99
```

Log out and log back in. Might be safer to simply reboot the payload computer. Finally, add a cron job:
```
crontab -e
```
And then add the following line to the end of the file:
```
@reboot bash ~/Drone-Project-code/Payload\ Computer/start_cal_v2.sh >~/cronlog 2>&1
```
To recover logs:
```
cat ~/cronlog
```
>__NOTE__: As of August 2020, there are several issues with USB performances on RPi running Ubuntu 20.04 and Python 3.0/GNU Radio 3.8. Going to revert to Ubuntu 18.04 and GNU Radio 3.7.13.4 for now.

[uhd]: https://files.ettus.com/manual/page_build_guide.html
[uhd2]: https://files.ettus.com/manual/page_install.html
[ROSberryPi_link]: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi
[noetic_install]: http://wiki.ros.org/noetic/Installation/Ubuntu
[uhd_install_from_git]: https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux