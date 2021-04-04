#!/bin/bash
# This script is designed to be run from inside the ESP32 docker.
set -ex

# Clone espidf repo.
if [ ! -e ~/ws/micro_ros_espidf_component ]
then
	cd ~/ws
	git clone -b foxy https://github.com/micro-ROS/micro_ros_espidf_component.git
fi

# Copy code from rover repo
cp -r ~/code/rover/raspi_rover ~/ws/micro_ros_espidf_component/examples
cp -r ~/code/raspi_robot_msgs ~/ws/micro_ros_espidf_component/extra_packages
cd ~/ws/micro_ros_espidf_component/examples/raspi_rover

# Build client code.
idf.py set-target esp32
idf.py menuconfig
idf.py build

echo
echo "Built rover client successfully.  To flash and monitor your code:"
echo "idf.py flash"
echo "idf.py monitor"
echo
