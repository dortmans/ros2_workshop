#!/bin/sh

IMAGE_NAME=dortmans/ros2
BASE_IMAGE=ros:foxy
DOCKER_USER=ros2

docker pull ${BASE_IMAGE}

docker build \
	--build-arg BASE_IMAGE=${BASE_IMAGE} \
	--build-arg USER=${DOCKER_USER} \
	--build-arg UID=$(id -u ${USER}) \
	--build-arg GID=$(id -g ${USER}) \
	-t ${IMAGE_NAME} .
	
docker image prune -f
