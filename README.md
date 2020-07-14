# Drone-Project
## Description
A high accuracy drone-based calibrator targeted towards low-frequency radio astronomy instruments. These are codes for the base station, payload computer, and for generating flight paths. More details inside each folder.

## Dependencies:
As of now, everything is tested on Ubuntu 18.04. Here are (some) of the additional dependencies. Follow instructions in the links for more.

### UHD and GNU Radio 
Follow instructions [here](https://kb.ettus.com/Building_and_Installing_the_USRP_Open-Source_Toolchain_(UHD_and_GNU_Radio)_on_Linux). Tested with UHD 3.13.0.0 and 3.15.0.0 and GNU Radio v3.7.13.4. NOTE: GNU Radio v3.8 and higher use Python 3.

### ROS Melodic + SITL toolchain
1. Start [here](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux_ubuntu.html). A convenience shell script will install ROS, Gazebo, MAVROS, Geographiclib and everything else. It will also set up a catkin workspace.
```
source ubuntu_sim_ros_melodic.sh
```
2. As of now, the Fast-DDS install within the above shell script is broken. Get the full install [here](https://www.eprosima.com/index.php/downloads-all).

3. Some additional dependencies that the shell script does not address (after getting repeated build errors):
```
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly –y
pip install geographiclib
```

4. PX4 firmware:
```
cd $HOME
mkdir /src/ && cd ~/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware/
make px4_sitl gazebo_solo
```
Full instructions to be found [here](https://dev.px4.io/v1.9.0/en/setup/building_px4.html). To run in headless mode:
```
HEADLESS=1 make px4_sitl gazebo_solo
```
5. Install QGroundControl. Detailed instructions [here](https://docs.qgroundcontrol.com/en/getting_started/download_and_install.html). Before downloading the app image do this:
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
## Autonomy Pipeline
The codes here enable complete autonomous calibration between the drone (SDR + payload computer) and the ground station (AUT + SDR + base station computer). Here's a flow diagram of the entire process:

![pipeline](autonomy_pipeline.jpg)

Once the PX4 firmware is up and running, this is how one could simulate the entire pipeline using SITL + Gazebo + ROS + GNURadio/UHD:
Terminal 1:
```
cd ~/src/Firmware/
HEADLESS=1 make px4_sitl gazebo_solo
```
Terminal 2:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
Terminal 3:
```
rosrun flying_spider drone_project.py
```
Terminal 4:
```
python ~/Drone-Project/Base\ Station/tcp_toggle.py
```
Terminal 5:
```
python ~/Drone-Project/Base\ Station/open-loop-accept-tcp.py
```
Run QGroundControl:
```
./Downloads/QGroundControl.AppImage
```
Enter flight path and begin mission.

#### NOTE: All of the above need to run in the exact sequence as shown.

#TODO: Add shell script that performs all of the above.

#TODO: Add the GNU Radio codes used for generating the calibration waveforms.

#TODO: Add the ROS nodes and mission.csv file/folders as well


