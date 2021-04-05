#!/bin/bash
# Stop the docker container.
set -e

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

if [ "$( docker container inspect -f '{{.State.Status}}' ${CONTAINER_NAME} )" == "running" ]
then
    # Container is running so stop it.
    docker stop ${CONTAINER_NAME} &> /dev/null
    echo "Docker '${CONTAINER_NAME}' stopped."
else
    echo "Docker '${CONTAINER_NAME}' already stopped."
fi

