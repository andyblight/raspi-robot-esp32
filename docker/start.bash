#!/bin/bash
# Start the docker container.
# $1 can be used to pass in a different code directory.
set -x

devices=""
ESP_DEVICE=/dev/ttyUSB0
JOYSTICK_DEVICE=/dev/input/js0
JOYSTICK_EVENT=/dev/ros2-joystick

usage() {
    echo
    echo "Usage: $0 [-h][-j][-e]"
    echo "Options:"
    echo "    -e    Start container with ESP32 on '${ESP_DEVICE}'."
    echo "    -j    Start container with joystick on '${JOYSTICK_EVENT}'."
    echo "Commands:"
    echo "    -h    Display this message."
    echo
}

check_esp32() {
    if [ -e ${ESP_DEVICE} ]
    then
        echo "ESP32 enabled."
        devices+="--device=${ESP_DEVICE} "
    else
        echo "${ESP_DEVICE} not found."
        echo "Please check that the ESP32 is connected and appears on the host as ${ESP_DEVICE}."
        usage
        exit 1
    fi
}

check_joystick() {
    if [ -e ${JOYSTICK_DEVICE} ]
    then
        if [ -e ${JOYSTICK_EVENT} ]
        then
            echo "Joystick enabled."
            devices+="--device=${JOYSTICK_EVENT} --device=${JOYSTICK_DEVICE} "
        else
            echo "${JOYSTICK_EVENT} not found."
            echo "Please check that the joystick is connected and appears on the host as ${JOYSTICK_EVENT}."
            usage
            exit 1
        fi
    else
        echo "${JOYSTICK_DEVICE} not found."
        echo "Please check that the joystick is connected and appears on the host as ${JOYSTICK_DEVICE}."
        usage
        exit 1
    fi
}

while getopts "ehj" opt; do
    case $opt in
        e)
            check_esp32
            ;;
        h)
            usage
            exit 0
            ;;
        j)
            check_joystick
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        :)
            echo "Option -$OPTARG requires an argument." >&2
            usage
            exit 1
            ;;
    esac
done

echo "Devices '$devices'"
# exit

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash
# Use the repo root dir for the code dir.
CODE_DIR=${docker_dir}/../..

mkdir -p ${WORKSPACE_DIR}
cp -f ${docker_dir}/setup*.bash ${WORKSPACE_DIR}
cp -f ${docker_dir}/build.bash ${WORKSPACE_DIR}

docker container inspect ${CONTAINER_NAME} &> /dev/null
if [ $? == 0 ]
then
    # Container exists.
    if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} )" == "running" ]
    then
        # Container is running.
        echo "Container '${CONTAINER_NAME}' is already running."
    else
        # Container exists but is not running.
        docker container start ${CONTAINER_NAME} &> /dev/null
        echo "Container '${CONTAINER_NAME}' started."
    fi
else
    # Container does not exist so run it.
    docker container run \
        --detach \
        --tty \
        --network=host \
        --name ${CONTAINER_NAME} \
        --env="DISPLAY=$DISPLAY" \
        --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
        --volume ${CODE_DIR}:/home/build/code \
        --volume ${WORKSPACE_DIR}:/home/build/ws \
        --volume="/etc/timezone:/etc/timezone:ro" \
        ${devices} \
        ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG} &> /dev/null
    if [ $? == 0 ]
    then
        echo "Container '${CONTAINER_NAME}' running."
    else
        echo "Container '${CONTAINER_NAME}' failed."
    fi
fi
