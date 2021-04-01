#!/bin/bash
# Configure the ESP32 workspace to use my ESP32 application.
# This is designed to be run inside the docker.
set -e

# Set up some variables.
APP_NAME=esp32_raspi_bot
IP_ADDRESS=192.168.2.28  # IP address of host PC running the agent.
APP_OPTIONS=" --transport udp --ip ${IP_ADDRESS} --port 8888 "

# Mounted by start.bash.
CODE_DIR=${HOME}/code
WS_DIR=${HOME}/ws

# Link the app code to the workspace.
cd ${WS_DIR}/firmware/freertos_apps/apps
UROS_APP_DIR=${CODE_DIR}/bots/ant/esp32_raspi_bot/app
ln -fs ${UROS_APP_DIR} ${APP_NAME}

# Link the driver and message packages.
cd ${WS_DIR}/firmware/mcu_ws
ln -fs ${CODE_DIR}/bots/ant/esp32_raspi_bot/raspi_robot_driver
ln -fs ${CODE_DIR}/bots/ant/esp32_raspi_bot/raspi_robot_msgs
ln -fs ${CODE_DIR}/pipebot_msgs

# Configure the app.
cd ${WS_DIR}
. ./install/setup.bash
ros2 run micro_ros_setup configure_firmware.sh ${APP_NAME} ${APP_OPTIONS}
