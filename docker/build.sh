#!/bin/sh

IMAGE_NAME=dortmans/ros2
DOCKER_USER=ros2

docker build \
	--build-arg USER=${DOCKER_USER} \
	--build-arg UID=$(id -u ${USER}) \
	--build-arg GID=$(id -g ${USER}) \
	-t ${IMAGE_NAME} .
