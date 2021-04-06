#!/bin/bash
# Commit the changes made to the container as a new image.
# Only works for one container running.
# set -x

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

# Work out the next tag number from the current container.
container_id=`docker container inspect -f '{{.Id}}' ${CONTAINER_NAME}`
image_name=`docker inspect --format='{{.Config.Image}}' ${container_id}`
# echo "Image ${image_name}"
image_tag=${image_name##*:}
# echo "Image tag ${image_tag}"
next_tag=$(($image_tag + 1))
# echo "Next tag ${next_tag}"

# Commit a new image.
docker container commit \
    ${CONTAINER_NAME} \
    ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${next_tag}

# Update vars.bash. Format 'IMAGE_TAG="1"' Replace the number.
sed -i "s/IMAGE_TAG=\".*\"/IMAGE_TAG=\"${next_tag}\"/g" ${docker_dir}/vars.bash
