## Payload computer programs
This folder contains all the codes needed to run the calibration signal using the single board computer on the drone payload.
* [This](drone_pulse_tx_single_pol_save_file.grc) GNU Radio flowgraph is used for generating a pre-stored waveform.
* Both the TCP [client](gr_cal_tcp_loopback_client.py) and [server](cal_sequence_tcp_server.py) files need to be added to cron jobs on the payload computer. The [qpsk_waveform](qpsk_Waveform) file needs to be in the same (working) directory.
* A shell script, [start_cal_v2.sh](start_cal_v2.sh) runs both at startup and saves a log of all output to the home directory.

Current candidates for the payload computer are: [Raspberry Pi 4B](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/), [NVIDIA Jetson TX2](https://developer.nvidia.com/embedded/jetson-tx2) with the [Orbitty carrier board](http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/), and an [ODROID XU4](https://www.hardkernel.com/shop/odroid-xu4-special-price/).

How to set things up on the payload computer. Clone the repo on the payload computer:
```
cd /home/$USER/
git clone https://github.com/km5es/Drone-Project.git
```
Add a cron job:
```
crontab -e
```
And then add the following line to the end of the file:
```
@reboot bash ~/Drone-Project/Payload\ Computer/start_cal_v2.sh >~/cronlog 2>&1
```
To recover logs:
```
cat ~/cronlog
```