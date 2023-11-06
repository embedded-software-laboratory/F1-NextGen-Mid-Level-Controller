# Mid Level Controller

This package contains the mid level controller for F1NextGen implemented in ROS2.
The mid level controller runs on the Raspberry Pi Zero W. It is part of the vehicle on-board electronics.

Its purposes are:

- Provide a synchronized clock on the vehicle.
- Provide a control system for accurate trajectory following of the vehicle.
- The control system runs locally on the vehicle to eliminate the network latency from the control loop.
- Send vehicle state information and receive trajectory commands via the network.

The full documentation for the mid level controller can be
found [here](https://cpm.embedded.rwth-aachen.de/doc/display/CLD/Mid+Level+Controller).

## Components

### Controllers

There are four ways to control the vehicles. They take precedence in the following order:

- Direct Commands
- Speed Curvature Commands
- Path Tracking Commands
- Trajectory Commands

That means if a vehicle receives both Direct Commands and Trajectory Commands, it will perform the Direct Commands.

This repository contains a controller for each command type that takes the control commands, converts them to the inputs
for the motor and servo and passes them to the low level controller.

### Localization

The localization combines the IPS vehicle observation with local vehicle sensor data to determine the vehicle pose. If
the IPS vehicle observation is delayed or temporarily unavailable, the localization can continue to give accurate poses
through dead reckoning. However, the dead reckoning will fail when driving aggressively, i.e. with high wheel slip.

### Sensor Calibration

The Low Level Controller provides its data as low-resolution integers. The LLC units are chosen to match the required
value range and quantization error. The sensor calibration converts the LLC data to floating point values with SI units.
