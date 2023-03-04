#!/bin/bash

CONTAINER_NAME=ros2
COMMAND=$*
COMMAND=${COMMAND:-/bin/bash}
COMMAND="/ros_entrypoint.sh $COMMAND"

docker exec -it \
  $CONTAINER_NAME \
  $COMMAND

