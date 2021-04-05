#!/bin/bash
# This script is designed to be run from inside the ESP32 docker.
set -ex


ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i [your local machine IP] -p 8888
ros2 run micro_ros_setup build_firmware.sh menuconfig

# Now go to the micro-ROS Transport Settings → WiFi Configuration menu and fill your WiFi SSID and password. Save your changes, exit the interactive menu, and run:
ros2 run micro_ros_setup build_firmware.sh

# Connect your ESP32 to the computer with a micro-USB cable, and run:
ros2 run micro_ros_setup flash_firmware.sh

echo
echo "Built rover client successfully.  To flash and monitor your code:"
echo "idf.py flash"
echo "idf.py monitor"
echo
