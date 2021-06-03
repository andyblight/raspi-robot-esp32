#!/bin/bash
set -e

# Copy code over.
rm -rf /home/build/ws/firmware/mcu_ws/raspi_robot_msgs/
cp -rf /home/build/code/raspi-robot-esp32/raspi_robot_msgs/ /home/build/ws/firmware/mcu_ws/
cp -rf /home/build/code/raspi-robot-esp32/raspi_robot_msgs/ /home/build/ws/src
cp -rf /home/build/code/raspi-robot-esp32/rover/raspi_rover/ /home/build/ws/firmware/freertos_apps/apps

# Build the new code.
cd /home/build/ws
. ./install/local_setup.bash
ros2 run micro_ros_setup build_firmware.sh
