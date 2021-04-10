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

* The ESP32 docker can be used to build, flash and monitor the ESP32. (3hrs)
* The Base station docker can be used to build the micro-ROS agent and ROS2 tools, e.g. RQt. (4hrs)
* Define custom messages to be used between rover and base station. (2hrs)
* Review stuff in other projects that might be useful for this project.  Especially, the 'esp32-raspi-robot' README files. (2hrs)
* Complete rover wiring. (14 hrs).
* Docker-client works for example code. (7hrs)
* Build existing software.  The method of building seems to have changed! (14 hrs).
  * Going back to old way of doing things.
    * Add script to install my code and build it.
    * Run test code.  Works with micro-ROS agent docker.
* Added battery status message. (8 hrs)
  * Battery monitoring uses voltage divider onto ADC pin.
    * Accuracy: -0.11V from voltmeter, -0.15 from charger, -0.20V from tester.
  * Std msg Int32 being used as BatteryState message causes crash.  Issue raised to deal with later.
  * Tested using micro-ROS agent docker.

Total hours: 54

## To do

* Connect the ESP32 to the base station docker and send messages both ways.
  * Get base station software working.
* Two identical dockers, base stations and rover client.  Can I use one?
* Implement each message/service and test.
  * Encoders
    * Add software for the encoders.
    * Implement publisher.
  * LEDs
    * Implemented. Needs testing from agent.
  * Motors
    * Implemented. Needs testing from agent.
  * Sonar
    * Implement service.
    * Add software to control the servos.
* Make the rover do something interesting!
* Define contents of diagnostic messages.
