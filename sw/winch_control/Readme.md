# Winch Documentation

The winch is a simple system used to control the length of the tether relative to the drones distance to the base.

## Hardware

The Hardware is composed of 
- Stepper Motor
- Motor driver
- Arduino Uno
- Spool on which the tether will wrap around
- Chain linking the stepper to the spool
- ITX computer on the Husky

## Software
There are two pieces of software, one for the Arduino Uno and another for the computer.

The winch controls the stepper according to the distance with the Husky.
To do that, it needs the drone coordinates. To get the coordinates we use the computer that will be connected to the whole systems network.
That allows us to use ROS2 and receive the coordinates from the RasberryPi and send them to the stepper motor driver thru a USB serial and the Arduino UNO.

### ITX

The ITX computer is running ubuntu. 
We have installed a docker container that launches a ROS2 node (`winch_controller.cpp`).
This node is a simple subscriber that receives the drones coordinates.
With those, it calculates the distance from the last recorded position to the new one, and sends out slack to the tether.

Most calculations are abstracted in the `winch.hpp` library file.
This file defines basic functions to send the correct distance to the winch.


### Arduino

`winch_control.ino`

The Arduino UNO R3 is the interface between the computer serial, and the stepper driver.
It receives, thru usb serial, a 2 bytes payload consisting of a direction (CW or CCW) and an amount (of steps) to move. It consequently sends the necessary amount of pulses to the stepper motor.

```
PAYLOAD :
x xxx xxxx xxxx xxxx
^ ^^^^^^^^^^^^^^^^^^
|            |
direction    |
             |
        steps to move
```

# Guide

## ITX

The docker should launch at startup anyways but here is how to launch manually:

- login to the ITX 
  - login: `administrator`
  - password: `clearpath`

- build the docker image with the dockerfile found in the `itx` directory, name it `custom-ros` or change the name in the launch command.

- launch it on the ITX with

```bash
docker run --rm --group-add 986 -w /home/cpp_pubsub --privileged -v /dev/:/dev/:rw -v ./cpp_pubsub:/home/cpp_pubsub:rw -it custom-ros:latest bash
```

- $ `source ./install/setup.bash`

- $ `ros2 launch cpp_pub_sub winch_controller`

⚠️ You may need to change the serial port in the `cpp_pubsub/src/winch.hpp` 

```c
#define USB_TARGET "/dev/ttyACM0"
```

## Arduino

- Choose which pins will control the `DIR` and `PUL` of the stepper driver in `winch_control.ino`.

```c
#define PUL_PIN 2
#define DIR_PIN 4
```

- Build and flash the sketch in the arduino every.

- Connect the right pins and plug it in the ITX and check it is connected to the same port as defined in the ros2 code.

## Stepper motor & driver

- Connect the stepper to its driver

- Connect the `PUL+` & `DIR+` to the defined pins and `PUL-` & `DIR-` to the ground, of the arduino.

- Connect the driver to 12-48V through the `GND` and `V+` ports

