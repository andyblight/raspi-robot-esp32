# RasPiRobot using ESP32

This is a personal project to breathe life into an old Raspberry Pi Rover project using an ESP32 CPU, ROS2 and micro-ROS.

The rover is controlled by an ESP32 which is fairly limited, so the ESP32 only controls the rover hardware and communicates with the base station using Wi-Fi.  The base station software runs on the PC inside a docker and is used to send commands to the rover and process data received from the rover's sensors.  This split was decided on so that the hardware and software can be developed separately.
