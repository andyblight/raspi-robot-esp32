# RasPiRobot using ESP32

This is a personal project to breathe life into an old Raspberry Pi Rover project using an ESP32 CPU, ROS2 and micro-ROS.

The rover is controlled by an ESP32 which is fairly limited, so the ESP32 only controls the rover hardware and communicates with the base station using Wi-Fi.  The base station software runs on the PC inside a docker and is used to send commands to the rover and process data received from the rover's sensors.  This split was decided on so that the hardware and software can be developed separately.

```text
----------------    ----------------    --------------
| Base station | <> | ROS Messages | <> | RasPiRover |
----------------    ----------------    --------------
```

## Project structure

The project structure is:

* base_station - the base station software.
  * docker - This docker has the full ROS2 desktop and basic graphics.  Includes the micro-ROS agent build.
* raspi_robot_messages - messages used to communicate between the rover and the base station.
* rover - software for the rover.
  * docker - This docker is command line only and is used purely to build and monitor the ESP32 firmware.
  * raspi_rover - the application files for the rover.
  * raspi_robot_driver - driver software for the RasPiRobot board.

## RasPi Robot Rover

The RasPi Robot Rover designed to be a flexible platform for testing ideas on.  To ensure that the code is easy to modify, the drivers on the rover are deliberately simple and are exposed to the base station so that the software can be rebuilt rapidly without having to rebuild and flash the code for the rover.  All rover messages and services are communicated with the base station using Wi-Fi.

The RasPi Robot Rover custom ROS messages and services are specified in the `raspi_robot_msgs` package.

In addition, the rover will publish diagnostic messages mainly for debugging purposes.

## Done

* The ESP32 docker can be used to build, flash and monitor the ESP32.
* The Base station docker can be used to build the micro-ROS agent and ROS2 tools, e.g. RQt.
* Define custom messages to be used between rover and base station.

## To do

* Build existing software.  The method of building seems to have changed!
* Review stuff in other projects that might be useful for this project.  Especially, the 'esp32-raspi-robot' README files.
* Connect the ESP32 to the base station docker and send messages both ways.
* Add software for the encoders.
* Add software for the servos.
* Implement each message/service and test.
  * Encoders
  * LEDs
  * Motors
  * Switches
  * Sonar
* Make the rover do something interesting!
* Battery monitoring - Voltage divider onto ADC pin?
* Define contents of diagnostic messages.
