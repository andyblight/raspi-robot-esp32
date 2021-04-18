#!/bin/bash
# Build the docker image.
# Not using set -e as `docker inspect` can legitimately fail.
# set -x

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
cd ${docker_dir}
. ./vars.bash

docker inspect --type=image ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG}  &> /dev/null
if [ $? == 1 ]
then
    set -e
    docker build \
        -t ${DOCKER_HUB_USER_NAME}/${IMAGE_NAME}:${IMAGE_TAG} \
        -f dockerfile \
        .
    set +e
fi

echo "Build of ${IMAGE_NAME} took $SECONDS seconds."
