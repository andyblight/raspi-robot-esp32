#!/bin/bash
set -e

git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
cd micro_ros_espidf_component/examples/int32_publisher

echo
echo "Now run the following commands"
echo ""
echo "cd micro_ros_espidf_component/examples/int32_publisher"
echo "idf.py set-target esp32"
echo "idf.py menuconfig"
echo "# Set your micro-ROS configuration and WiFi credentials under micro-ROS Settings"
echo "idf.py build"
echo "idf.py flash"
echo "idf.py monitor"
echo
