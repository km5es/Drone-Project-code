# Payload computer 

This folder contains all the code that needs to be run on the payload computer. The payload computer is a Raspberry Pi 4B (4 GB variant) running 32-bit Raspbian (Buster). Broadly speaking, its dependencies are the same as the [base station](../).

> **NOTE:** The complete image with all the scripts and dependencies installed can be found [here](https://drive.google.com/file/d/1IdIcXCnkl9QE6W4AtAGRXWZ7IAf_Bb1v/view?usp=sharing) (size: ~14.5 GB, 32 GB SD card required). Login details are available on request.



## Overview of how it works

The Pi is interfaced to the flight controller and to the software-defined radio (SDR) via serial connections. Several ROS nodes instruct the flight controller where to navigate and ensure process synchronization. Upon reaching each waypoint, the SDR will begin transmitting a pre-stored waveform on loop for a set duration. During this time, it will also save metadata to the [`./logs`](../logs/) folder for use in post-processing and analysis. 

- The [`write_WPs.py`](./write_WPs.py) node is responsible for writing waypoints to the flight controller. Waypoint list is retrieved from [`./mission/mission.waypoints`](../mission/mission.waypoints).
- [`wp_trigger.py`](./wp_trigger.py) looks to see when the drone has reached a WP, is stable not moving too much, and then sets a trigger flag. 
- [`cal_sequence_tcp_server_v2.py`](./cal_sequence_tcp_server_v2.py) identifies when the flag has been set and begins feeding the pre-stored waveform to the SDR. It also triggers the metadata flag.
- Finally, the [`get_metadata.py`](./get_metadata.py) receives metadata flag and begins saving metadata like SDR temperature, drone GPS position, and IMU data to separate files along with timestamps.

All of these nodes are folded into shell scripts which will run on bootup. Details can be found in the cron folder by typing `sudo crontab -e`. 



## Generating a waveform

This folder contains several GNU Radio flowgraphs for generating a variety of pre-stored waveforms. The default waveform is a pulsed sine wave at 960 kHz baseband. Other waveforms such as a pulsed Gaussian-modulated sine wave, QPSK-modulated waveform, and broadband noise are also possible with the attached GR flowgraphs. 

> **Warning:** When the SDR is not transmitting the calibration signal it will feed a sequence of zeros to its front-end. This will result in a low-amplitude broadband noise emission. 



## Logging

- Logs from all of the navigation and SDR scripts are saved in the [`./logs/payload/`](../logs/payload/) folder. These are useful to identify if/when calibrations took place on the payload side.
- Metadata logs are saved in [`./logs/metadata/`](../logs/metadata/). Required for post-processing and analysis. 
- In addition, the stdout from the `cron` jobs are saved in the [`./logs`](../logs/) folder for debugging issues with the scripts booting up automatically. 
- Finally, all ROS nodes are initialized as such and logging output from those are saved in standard ROS format in the `~/.ros/log` folder as well.



