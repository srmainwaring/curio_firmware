# Firmware for Curio

An Arduino Mega 2560
[`rosserial_arduino`](http://wiki.ros.org/rosserial_arduino)
client for controlling a Sawppy rover.
To be used in conjunction with the
[`curio`](https://github.com/srmainwaring/curio) packages.

## Overview

The firmware implements an instance of a
[`rosserial_client`](http://wiki.ros.org/rosserial_client)
that runs on the Arduino and is managed by a
[`rosserial`](http://wiki.ros.org/rosserial?distro=melodic) node
running on the rover's onboard computer.

The firmware provides:

- A base controller to control the rover's servos
- A decoder for a radio control receiver

Both functions can be enabled or disabled by compile time switches,
and the radio control decoder can be configured to process either
pulse width modulated signals or a radio control serial communication
protocol called SUMD.

## Dependencies

- [`rosserial_python`](
http://wiki.ros.org/rosserial_arduino) - manages the host-side of
the serial connection and communication to other ROS nodes.
- [`rosserial_arduino`](http://wiki.ros.org/rosserial_arduino) -
provides the client Arduino ROS interface: publish and subscribe messages,
access parameters, etc.
- [`curio_msgs`](https://github.com/srmainwaring/curio_msgs) -
define the message format used for controlling servos and publishing
radio control PWM data.
- [EnableInterrupt](https://github.com/GreyGnome/EnableInterrupt) -
used by the PWM decoder to handle interrupts for each of the RC channels.

## Installation

Here we describe how to build and install the firmware to your Arduino.
In the following SKETCHBOOK_PATH is the path to your Arduino
sketchbook directory.

Clone the repository into your catkin workspace, then build:

```bash
cd ~/curio_ws/src
git clone https://github.com/srmainwaring/curio_firmware.git
cd ~/curio_ws
catkin build
```

Build and install the ROS libraries for the Arduino:

```bash
cd ~/curio_ws
rosrun rosserial_arduino make_libraries.py .
cp -rp ./ros_lib SKETCHBOOK_PATH/libraries
```

Add a file `ros_dummy.h` to the ROS Arduino library in `SKETCHBOOK_PATH/libraries/ros_lib/` containing the following:

```c++
// ros_dummy.h
// A placeholder so that the Arduino IDE can find header files
// in subdirectories of ros_lib when re-declaring ros.h 
// in your project
//
// In your copy of ros.h include this file. 
//
// Resources:
//  http://wiki.ros.org/rosserial_arduino/Tutorials/NodeHandle%20and%20ArduinoHardware
//  https://www.codeproject.com/Articles/1279552/Rodney-A-Long-Time-Coming-Autonomous-Robot-Part-7

#ifndef _ROS_LIB_ROS_DUMMY_H_
#define _ROS_LIB_ROS_DUMMY_H_

namespace ros {
}

#endif // _ROS_LIB_ROS_DUMMY_H_
```

(My thanks to [Phil Hopley's excellent blog](https://www.codeproject.com/Articles/1279552/Rodney-A-Long-Time-Coming-Autonomous-Robot-Part-7) for his explanation of why this is is needed).

Copy the firmware source to the sketchbook folder:

```bash
cp -rp `rospack find curio_firmware`/firmware/curio_firmware SKETCHBOOK_PATH/curio_firmware
```

From the Arduino IDE open the sketch and adjust configuration settings
described in the section below as required. Finally compile the sketch
and download it to your Arduino.

## Configuration

The main sketch file `curio_firmware.ion` contains the following
configuration flags:

`ENABLE_ARDUINO_LX16A_DRIVER`

- `1`: Enable the Arduino LX-16A driver.
The Arduino will subscribe to servo commands and publish unfiltered
servo positions.
- `0`: Disable the Arduino LX-16A driver.
All servo control is carried out by the Python base controller.

Note that the corresponding flag in `curio_base/src/base_controller.py`
must also be set.

`ENABLE_RADIO_CONTROL_DECODER`

- `1`: Enable the radio control decoder. The Arduino will publish
radio control PWM values on a `curio_msgs/Channels` message.
- `0`: Disable the radio control decoder.

`ENABLE_RADIO_CONTROL_SUMD`

- `1`: Use the SUMD decoder.
- `0`: Use the PWM decoder.

## Hardware

 The following wiring instructions apply to the Arduino Mega 2560.

### Lewansoul BusLinker

- `5V` on the BusLinker to Arduino `+5V`
- `GND` on the BusLinker to Arduino `GND`
- `TX` on the BusLinker to Arduino `RX1` PIN 19
- `RX` on the BusLinker to Arduino `TX1` PIN 18

### Radio control receiver

#### SUMD

- `+` on the receiver to Arduino `+5V`
- `-` on the receiver to Arduino `GND`
- `S` channel 5 on the receiver to Arduino `RX2` PIN 17

#### PWM

- `+` on the receiver to Arduino `+5V`
- `-` on the receiver to Arduino `GND`
- `S` channel 1 on the receiver to Arduino PIN 2
- `S` channel 2 on the receiver to Arduino PIN 3
- `S` channel 3 on the receiver to Arduino PIN 4
- `S` channel 4 on the receiver to Arduino PIN 5
- `S` channel 5 on the receiver to Arduino PIN 6
- `S` channel 6 on the receiver to Arduino PIN 7

## Usage

See the documentation for [`curio`](https://github.com/srmainwaring/curio.git).

## ROS

The parameters, publications and subscriptions supported
are affected by the configuration settings. For instance if radio control
is disabled the `curio_msgs::Channels` messages will not be published.

### Parameters

- ~wheel_servo_ids : list\
    A list of integer wheel servo ids, has (default [11, 12, 13, 21, 22, 23])
- ~steer_servo_ids : list\
    A list of integer steering servo ids, has (default [111, 131, 211, 231])
- ~steer_servo_angle_offsets : list\
    A list of integer steering servo angle offsets for adjusting steering trim,
    has (default [0, 0, 0, 0]

### Publications

- servo/positions : curio_msgs/CurioServoPositions\
    The wheel and steering servo positions (rate 30Hz)
- servo/states : curio_msgs/CurioServoStates\
    The diagnostic state of each servo (rate 1Hz)
- radio/channels : curio_msgs/Channels\
    PWM values for one or more radio control channels

### Subscriptions

- servo/commands : curio_msgs/CurioServoCommands\
    The commanded duty and position for the wheel and steering servos

## License

This software is licensed under the BSD-3-Clause license found in the LICENSE file
in the root directory of this source tree.
