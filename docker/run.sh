#!/bin/sh

IMAGE_NAME=dortmans/ros2
COMMAND=$*
DOCKER_USER=ros2

docker run -it --rm \
  -e DISPLAY \
  -e XAUTHORITY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/shm:/dev/shm \
  -v /home/$USER/:/home/$DOCKER_USER/ \
  --device=/dev/dri \
  --privileged --net=host \
  --gpus all \
  $IMAGE_NAME $COMMAND
