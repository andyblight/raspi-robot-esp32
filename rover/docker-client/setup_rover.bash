#!/bin/bash
# This script is designed to be run from inside the ESP32 docker.
set -ex

# Ensure ESP32 basics are present.
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32

# Link up rover code.
cd ~/ws
# It is OK to link the app directory.
ln -s ~/code/rover/app/ firmware/freertos_apps/apps/raspi_rover
# This has to be copied otherwise the build system does not recognise it.
cp -r ~/code/raspi_robot_msgs/ firmware/mcu_ws

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
