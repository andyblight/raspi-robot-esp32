# RasPiRobot using ESP32

This is a personal project to breathe life into an old Raspberry Pi Rover project using an ESP32 CPU, ROS2 and micro-ROS.

## Project structure

The rover is controlled by an ESP32 which is fairly limited, so the ESP32 only controls the rover hardware and communicates with the base station using Wi-Fi.  The base station software runs on the PC inside a docker and is used to send commands to the rover and process data received from the rover's sensors.  This split was decided on so that the hardware and software can be developed separately.

```text
-----------------------------------------------------------       ----------------------------------------  
| Base station                                            |       | Rover                                |
| -----------------    ----------------    -------------- | Wi-Fi | ---------------    ----------------- |
| | ROS processes | <> | ROS Messages | <> | uROS Agent | <======>| | uROS Client | <> | Rover Drivers | |
| -----------------    ----------------    -------------- |       | ---------------    ----------------- |
-----------------------------------------------------------       ---------------------------------------- 
```

The project software structure shown above is roughly reflected in the directory structure:

* docker - A docker based on Ubuntu server 20.04LTS that has:
  * The micro-ROS agent.
  * The micro-ROS client. NOTE: The client has to be build in the same workspace as the agent.
  * Basic display X window capabilities for tools such as RQt. 
  * Other ROS2 packages needed to control the robot.
* raspi_robot_messages - messages used to communicate between the rover and the base station.
* rover - software for the rover.
  * raspi_rover - the application files for the rover.
  * raspi_robot_driver - driver software for the RasPiRobot board.

## RasPi Robot Rover

The RasPi Robot Rover designed to be a flexible platform for testing ideas on.  To ensure that the code is easy to modify, the drivers on the rover are deliberately simple and are exposed to the base station so that the software can be rebuilt rapidly without having to rebuild and flash the code for the rover.  All rover messages and services are communicated with the base station using Wi-Fi.

The RasPi Robot Rover custom ROS messages and services are defined in the `raspi_robot_msgs` package.

In addition, the rover will publish some diagnostic messages for debugging purposes.

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
* Build agent using raspi_robot_msgs.
    Got bask to a working setup.
    Agent builds and runs.
    Added `raspi_robot_msgs` to agent (this took a while, see build_system.md for details).
  * LEDs
    * Implemented. Needs testing from agent. Works.
  * Motors
    * Implemented. Needs testing from agent. Works.
  2hrs
* Connect the ESP32 to agent docker and send messages both ways.  DONE.
  * Two identical dockers, base stations and rover client.  Can I use one?
    YES. Only one workspace needed.
    Remove base station docker. DONE. 
    Move client docker and rename. DONE.
  30mins.
* 

Total hours: 56

## To do

* Implement diagnositic message.  
  * To do bare minimum.  Just report string values.

To contain:
  * Encoder pulses.
  * Motor drive values.
  * Publish on diagnostic message.
* Implement each message/service and test.
  * Encoders
    * Add driver for the encoders.
  * Sonar
    * Add driver for servos. Must support multiple instances.
    * Implement service to set position and report distance. 
* Make the rover do something interesting!
* Define contents of diagnostic messages.
* Decide on use of standard messaegs for rover.
  * `cmd_vel` to move.
    Will need PID controller as the motors have different characteristics.
  * `odom` to publish position change based on encoder data.
    Distance = revolutions * circumference of wheel.




* Test Arduino style build for ESP32.  
<https://discourse.ros.org/t/micro-ros-porting-to-esp32/16101/13>
<https://github.com/micro-ROS/micro_ros_espidf_component/issues/9>
