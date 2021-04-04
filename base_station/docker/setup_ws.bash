#!/bin/bash
# Set up the workspace for ESP32 build.
# This is designed to be run from inside the docker.
set -e

cd ${HOME}/ws
mkdir -p src
if [ ! -e src/micro_ros_setup ]
then
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
fi

# Update dependencies using rosdep
. /opt/ros/foxy/setup.bash
sudo apt update
rosdep update
rosdep install --from-path src --ignore-src -y

# Build microros so we can run the create_agent_ws.sh.
cd ${HOME}/ws
colcon build --packages-select micro_ros_setup
. ./install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

echo
echo "Setup took $SECONDS seconds."
echo
