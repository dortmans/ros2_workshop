#!/bin/sh

IMAGE_NAME=dortmans/ros2
CONTAINER_NAME=ros2
DOCKER_USER=ros2

NVIDIA_GPU_SETTINGS="--gpus all"
OTHER_GPU_SETTINGS=""
OLD_GPU_SETTINGS="-e LIBGL_ALWAYS_SOFTWARE=1"

GPU_SETTINGS=$NVIDIA_GPU_SETTINGS

docker run -d -it --rm \
  -e DISPLAY \
  -e XAUTHORITY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/shm:/dev/shm \
  -v /home/$USER/:/home/$DOCKER_USER/ \
  --device=/dev/dri \
  --privileged --net=host \
  $GPU_SETTINGS \
  --name $CONTAINER_NAME \
  $IMAGE_NAME
