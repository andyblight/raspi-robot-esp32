#!/bin/bash
# Based on https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/
# and https://micro.ros.org/blog/2020/08/27/esp32/
set -e

# Create a workspace and download the micro-ROS tools
mkdir -p ~/ws/src
cd ~/ws/src
if [ ! -e ~/ws/src/micro_ros_setup ]
then
	git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
fi

# Source the ROS 2 installation
source /opt/ros/foxy/setup.bash

# Update dependencies using rosdep
sudo apt update
rosdep update
cd ~/ws
rosdep install --from-path src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash

# Run micro-ROS tools to setup application for ESP32.
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i 192.168.1.1 -p 8888

echo
echo "Now use this command:"
echo "ros2 run micro_ros_setup build_firmware.sh menuconfig"
echo "to setup the IP address of the host PC, Wi-Fi SSID and password."
echo
echo "Then build and flash using:"
echo "ros2 run micro_ros_setup build_firmware.sh"
echo "ros2 run micro_ros_setup flash_firmware.sh "
echo
