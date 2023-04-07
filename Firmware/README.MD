# Firmware
Arducopter `v4.1.0` and above cause PRIAM to become critically unstable in Pos Hold or Loiter modes due to changes in the attitude controller. This makes the drone unflyable in Auto mode.

All of my trials and tribulations with this problem have been chronicled on the Ardupilot forums [here][].

I have performed several flights with Copter `v4.0.5` and a Pixhawk 2.4.8. It has been stable thus far. With a Cube Orange, the drone is not *critically* unstable but not safe enough to fly either. It is possible this problem would be fixed in the future but for now, this folder contains the older firmware for posterity.

[//]: <> (TODO: add parameters file as well.)

[here]: https://discuss.ardupilot.org/t/oscillate-and-flip-over-after-switching-fc/85751