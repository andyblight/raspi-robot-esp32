#!/bin/bash
# Remove the docker container and image.
# set -e

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

# Delete the image.
image_tag="${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG}"
image_id=`docker image inspect -f '{{.Id}}' ${image_tag}`
if [ $? == 0 ]
then
    docker image rm -f ${image_id}
    echo "Docker image '${image_tag}' removed."
fi
