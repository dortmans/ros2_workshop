#!/bin/bash

IMAGE_NAME=dortmans/ros2
ROS_DISTRO=humble # foxy, humble, rolling
BASE_IMAGE=osrf/ros:${ROS_DISTRO}-desktop
DOCKER_USER=ros2

docker pull ${BASE_IMAGE}

docker build \
	--build-arg ROS_DISTRO=${ROS_DISTRO} \
	--build-arg USER=${DOCKER_USER} \
	--build-arg UID=$(id -u ${USER}) \
	--build-arg GID=$(id -g ${USER}) \
	-t ${IMAGE_NAME} .
	
docker image prune -f
