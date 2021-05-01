#!/bin/bash
# This script is designed to be run from inside the ESP32 docker.
set -ex

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
source ./install/local_setup.bash

# Run micro-ROS tools to setup application for ESP32.
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# Copy and build custom messages
rm -rf ~/ws/firmware/mcu_ws/raspi_robot_msgs/
cp -r ~/code/raspi-robot-esp32/raspi_robot_msgs/ ~/ws/firmware/mcu_ws/
cd ~/ws
colcon build --packages-select raspi_robot_msgs

# Apply patch to CMakeLists.txt file so it picks up the app.cmake file.
cd ~/ws/firmware/freertos_apps
git apply ~/code/rover/docker-client/CMakeLists.patch

# Copy rover code into workspace.
cp -r /home/build/code/raspi-robot-esp32/rover/raspi_rover/ /home/build/ws/firmware/freertos_apps/apps

# Build the new code.
ros2 run micro_ros_setup configure_firmware.sh raspi_rover -t udp -i 192.168.1.1 -p 8888

echo
echo "Now use this command:"
echo "ros2 run micro_ros_setup build_firmware.sh menuconfig"
echo "to setup the IP address of the host PC, Wi-Fi SSID and password."
echo
echo "Then build and flash using:"
echo "ros2 run micro_ros_setup build_firmware.sh"
echo "ros2 run micro_ros_setup flash_firmware.sh "
echo
