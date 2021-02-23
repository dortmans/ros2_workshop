#!/bin/bash
#
# Run ROS2 desktop in browser.
#
# Author: eric.dortmans@gmail.com

DISTRO=foxy #latest
if [ -n "${1}" ]; then
	DISTRO=${1}
fi

WORKSPACE=ros2_ws

# Update image
docker pull tiryoh/ros2-desktop-vnc:$DISTRO

# Start image
echo "Browse http://localhost/ to access the desktop"
docker run --rm \
	--net=host \
	-v /dev/shm:/dev/shm \
	-v /etc/localtime:/etc/localtime:ro \
	-v /home/$USER/:/home/ubuntu/ \
	tiryoh/ros2-desktop-vnc:$DISTRO

