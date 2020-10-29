## Base station
### Setting up the base station:
>**NOTE**: The current build only works with Ubuntu 18.04, ROS Melodic, UHD 3.15.0.0, and GNU Radio 3.7.13.4.

The tcp_toggle.py file has a multi_usrp block which writes data via tcp loopback to the base-station-receiver.py file which has a ROS node built-in. 
ROS node looks for drone position, begins saving data from tcp, and triggers the payload computer. 

There are multiple serial radios that are used for enabling the autonomy pipeline. To avoid confusion, and to define in advance which serial radio does what one needs to add symlinks to each serial device. This is done by creating a new udev rules file. 

Start by creating a new file in `/etc/udev/rules.d/99-pixhawk.rules`. Then connect each serial radio and find its info by:
```
udevadm info /dev/ttyUSB0 | grep 'VENDOR_ID\|ID_MODEL_ID\|ID_SERIAL_SHORT'
```
An example output looks like this:
```
E: ID_MODEL_ID=6015
E: ID_SERIAL_SHORT=DN01GAMJ
E: ID_VENDOR_ID=0403
```
Enter those data in `/etc/udev/rules.d/99-pixhawk.rules` like so:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DN01GAMJ", SYMLINK+="ttyPAYLOAD"
```
The other radios can be entered into symlinks as well like so:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="D308M5SK", SYMLINK+="ttyPAYLOAD"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{serial}=="DN01GAMJ", SYMLINK+="ttyDRONE"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="018E1FA2", SYMLINK+="ttyMAVROS"

```
Note, that you would have to reboot the computer for this to take effect. Once that is done, each of these serial objects can be uniquely mapped/called in the scripts using, for example, `/dev/ttyPAYLOAD` instead of getting confused with `/dev/ttyUSB0`.

Finally, to run the code:
```
sh ~/catkin_ws/src/Drone-Project-code/Base\ Station/begin_mission.sh
```
Detailed event logs are saved in `./logs` folder (relative to main git repo dir). The filenames contain timestamps so each time the above shell script is initiated there will be a new log file created. 


>**NOTE:** ttyDRONE is the radio connected to TELEM1 on the FC, ttyMAVROS is the one connected to TELEM2 and ttyPAYLOAD is connected to the payload computer. These have to be set up in advance using [3DRradioconfig.exe][] to ensure they do not interfere with each other.


**References:**

https://dev.px4.io/v1.9.0/en/companion_computer/pixhawk_companion.html\
https://www.tecmint.com/udev-for-device-detection-management-in-linux/\
https://linuxconfig.org/tutorial-on-how-to-write-basic-udev-rules-in-linux


[3DRradioconfig.exe]: http://vps.oborne.me/3drradioconfig.zip