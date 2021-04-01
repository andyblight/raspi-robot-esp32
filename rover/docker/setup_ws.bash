#!/bin/bash
# Set up the workspace for ESP32 build.
# This is designed to be run from inside the docker.
set -e

cd ${HOME}/ws
if [ ! -e src/micro_ros_setup ]
then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi
sudo apt update
. /opt/ros/foxy/setup.bash
colcon build
. install/local_setup.bash
ros2 run micro_ros_setup create_firmware_ws.sh freertos esp32
ros2 run micro_ros_setup configure_firmware.sh int32_publisher -t udp -i 192.168.0.100 -p 8888
ros2 run micro_ros_setup build_firmware.sh

echo "To build again, run these commands:"
echo ". install/setup.bash"
echo "ros2 run micro_ros_setup build_firmware.sh"
echo
echo "Setup took $SECONDS seconds."
