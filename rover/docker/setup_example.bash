#!/bin/bash
set -e

cd ~
if [ ! -e ~/ws/micro_ros_espidf_component ]
then
	git clone -b foxy https://github.com/micro-ROS/micro_ros_espidf_component.git
fi

cd ~/ws/micro_ros_espidf_component/examples/int32_publisher
idf.py set-target esp32
idf.py menuconfig
idf.py build

echo
echo "Built example successfully.  To flash and monitor your code:"
echo "idf.py flash"
echo "idf.py monitor"
echo
