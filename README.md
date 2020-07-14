# Drone-Project
## Description
A high accuracy drone-based calibrator targeted towards low-frequency radio astronomy instruments. These are codes for the base station, payload computer, and for generating flight paths. More details inside each folder.

## Dependencies:
As of now, everything is tested on Ubuntu 18.04. Here are (some) of the additional dependencies. Follow instructions in the links for more.
1. UHD and GNU Radio. Follow instructions [here](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux). Tested with UHD 3.13.0.0 and 3.15.0.0 and GNU Radio v3.7.13.4. NOTE: GNU Radio v3.8 and higher use Python 3.
2. ROS Melodic + SITL software:
a. Start [here](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html)
A convenience shell script will install ROS, Gazebo, MAVROS, Geographiclib and everything else:
```
source ubuntu_sim_ros_melodic.sh
```
As of now, the Fast-DDS install is broken. Get the full install [here](https://www.eprosima.com/index.php/downloads-all)

b. Some additional dependencies that the shell script does not address (after getting repeated build errors):
```
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly –y
pip install geographiclib
```

c. PX4 firmware:
```
cd $HOME
mkdir /src/ && cd ~/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware/
make px4_sitl gazebo_solo
```
To run in headless mode:
```
HEADLESS=1 make px4_sitl gazebo_solo
```

## Autonomy Pipeline
[pipeline](autonomy_pipeline.jpg)
The codes here enable complete autonomous calibration between the drone (SDR + payload computer) and the ground station (AUT + SDR + base station computer). 