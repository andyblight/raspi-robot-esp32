#!/bin/bash
# This script is designed to be run from inside the ESP32 docker.
set -ex

# Create a workspace and download the micro-ROS tools
mkdir -p ~/ws/src
cd ~/ws/src
if [ ! -e ~/ws/src/micro_ros_setup ]
then
	echo "Please run 'setup_rover.bash' first."
	exit 1
fi

# Copy and build the custom messages package.
cd ~/ws/src
cp -r ../firmware/mcu_ws/raspi_robot_msgs/ .
cd ~/ws
colcon build --packages-select raspi_robot_msgs
. install/local_setup.bash

# TODO Add more stuff for the base station here.

# Setup and build the agent.
sudo apt update
rosdep update
. /opt/ros/foxy/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
. install/local_setup.bash

echo
echo "To run the agent:"
echo "ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo
echo "To rebuild the agent:"
echo "ros2 run micro_ros_setup build_agent.sh"
echo ". install/local_setup.bash"
echo
