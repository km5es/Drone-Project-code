# Drone-Project-code
<img src="calibration_animation.gif" width=1000 align=center>

## Description
The Precision Radio Instrument for Antenna Measurements (PRAM) is a high accuracy drone-based calibrator targeted primarily towards low-frequency radio astronomy instruments. These are codes for the base station, payload computer, and for generating flight paths. The following README will detail how to install dependencies on the base station computer and run the SITL simulations. These simulations combine the SDR code with ROS and the flight stack to make the calibration autonomous. More details inside each folder. The repo containing CAD files, PCB designs and RF simulations lives [here](https://github.com/km5es/Drone-Project-CAD.git). 

> **Note**: A complete [Gitbook](app.gitbook.com) guide is in the works and will be put up here duly.

Start by cloning the repo into a catkin workspace:
```
mkdir /home/$USER/catkin_ws/src/ -p
cd /home/$USER/catkin_ws/src
git clone https://github.com/km5es/Drone-Project-code.git
```

## Dependencies:
As of now, these are the most pertinent top-level software dependencies:
- Ubuntu 18.04
- Python 2.7
- [Ardupilot](https://ardupilot.org/) firmware for the flight controller.

Here are (some) of the additional dependencies. Follow instructions in the links for more. Payload computer dependencies and instructions are provided [here](./Payload%20Computer).

### UHD and GNU Radio 
Follow instructions [here](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux). Tested with UHD 3.13.0.0 and 3.15.0.0 and GNU Radio v3.7.13.4. 
> **Note**: GNU Radio v3.8 and higher use Python 3. A complete migration to Python 3 is TBD. 

### ROS Melodic + SITL toolchain

#### 1. A convenience shell script will install ROS, Gazebo, MAVROS, Geographiclib and everything else. It will also set up a catkin workspace.
```
cd /home/$USER/catkin_ws/src/Drone-Project-code/
source ubuntu_sim_ros_melodic.sh
```

#### 3. Some additional dependencies that the shell script does not address (after getting repeated build errors):
```
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly –y
pip install geographiclib termcolor && pip3 install geographiclib termcolor 
```

#### 4. Ardupilot firmware:
Follow instructions [here](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html). To set a custom start location for the simulation edit the ```/home/$USER/.config/ardupilot/locations.txt``` file like so:
```
#NAME=latitude,longitude,absolute-altitude,heading
MiltonAirfield=37.994125,-78.397535,28.5,90
```
If ```locations.txt``` does not exist, create it. Change coordinates and the name of the starting location as necessary.

Install Geographiclib datasets:
```
cd ~/catkin_ws/src/Drone-Project-code/
chmod +x install_geographiclib_datasets
sudo ./install_geographiclib_datasets
```

Install MAVROS:
```
sudo apt-get install ros-melodic-mavros
```
Install mavlink-routerd by following instructions [here](https://github.com/mavlink-router/mavlink-router). 

#### 5. Install QGroundControl. Detailed instructions [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html). Before downloading the app image do this:
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
Log out and log back in. Then download the app image file from [here](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage).
```
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
```

#### 6. Build the ROS nodes:
```
cd /home/$USER/catkin_ws/
catkin_make
```