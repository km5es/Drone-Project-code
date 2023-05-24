# Autopilot firmware
PRIAM is critically unstable in any flight mode that requires position control (`PosHold`, `Loiter`, and `Auto` for example) in Copter versions >`v4.1`. This issue has been documented in detail on the ArduCopter message boards [here](https://discuss.ardupilot.org/t/oscillate-and-flip-over-after-switching-fc/85751). It has been tested extensively on `Copter v4.0.5` and been found to be stable. This folder contains the firmware for the Pixhawk 2.4.8 and the CubeOrange autopilots. 

**Warning that the latter is *not* stable either in those modes even on `Copter v4.0.5`.**