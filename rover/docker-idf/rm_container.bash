#!/bin/bash
# Remove the docker container and image.
# set -e

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

# Remove container.
container_id=`docker container inspect -f '{{.Id}}' ${CONTAINER_NAME}`
if [ $? == 0 ]
then
    docker container rm -f ${container_id}
    echo "Docker container '${CONTAINER_NAME}' removed."
fi
