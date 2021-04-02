#!/bin/bash
# Start the docker container.
# $1 can be used to pass in a different code directory.
# set -x

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash
# Use the repo root dir for the code dir.
CODE_DIR=${docker_dir}/../..

mkdir -p ${WORKSPACE_DIR}
cp ${docker_dir}/setup_*.bash ${WORKSPACE_DIR}

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
        --net=host \
        --name ${CONTAINER_NAME} \
        --volume ${CODE_DIR}:/home/build/code \
        --volume ${WORKSPACE_DIR}:/home/build/ws \
        --volume="/etc/timezone:/etc/timezone:ro" \
        --device=/dev/ttyUSB0 \
        ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG} &> /dev/null
    if [ $? == 0 ]
    then
        echo "Container '${CONTAINER_NAME}' running."
    else
        echo "Container '${CONTAINER_NAME}' failed."
        echo "Please check that the ESP32 is connected and appears on the host as /dev/ttyUSB0."
    fi
fi

#
# docker run -it --rm --user espidf \
# --volume="/etc/timezone:/etc/timezone:ro" \
# -v  $(pwd):/micro_ros_espidf_component \
# -v  /dev:/dev \
# --privileged \
# --workdir /micro_ros_espidf_component \
# microros/esp-idf-microros:latest \
# /bin/bash  -c "cd examples/int32_publisher; idf.py menuconfig build flash monitor"
