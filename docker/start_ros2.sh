#!/bin/bash

IMAGE_NAME=dortmans/ros2
CONTAINER_NAME=ros2
DOCKER_USER=ros2

NVIDIA_GPU_SETTINGS="--gpus all -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=graphics"
OTHER_GPU_SETTINGS=""
OLD_GPU_SETTINGS="-e LIBGL_ALWAYS_SOFTWARE=1"

GPU_SETTINGS=$OTHER_GPU_SETTINGS

docker run -d -t --rm \
  -e DISPLAY \
  -e XAUTHORITY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro \
  -v /home/$USER/:/home/$DOCKER_USER/ \
  -v /dev:/dev \
  --privileged --net=host \
  $GPU_SETTINGS \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
  

