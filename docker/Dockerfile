ARG ROS_DISTRO
ARG BASE_IMAGE=osrf/ros:$ROS_DISTRO-desktop
FROM $BASE_IMAGE

ENV DEBIAN_FRONTEND=noninteractive

# Install extra packages
RUN apt-get update && apt-get install -y \
    mesa-utils pciutils \
    iputils-ping net-tools tcpdump iperf \
    wget curl gnupg2 lsb-release ccache coreutils \
    vim-tiny \
    python3-pip \
    ros-$ROS_DISTRO-image-common \
    ros-$ROS_DISTRO-image-pipeline \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-laser-geometry \
    ros-$ROS_DISTRO-perception-pcl \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros-ign \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-teleop-tools \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$ROS_DISTRO" != "foxy" ] ; then \
		apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-laser-filters \
    && rm -rf /var/lib/apt/lists/* \
    ; fi
    
# Create docker user
ARG USER
ARG UID
ARG GID
RUN useradd -m $USER && \
	  echo "$USER:$USER" | chpasswd && \
    usermod --shell /bin/bash $USER && \
    usermod -aG dialout $USER && \
    usermod -aG video $USER && \
    usermod -aG sudo $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    usermod  --uid $UID $USER && \
    groupmod --gid $GID $USER
USER $USER
WORKDIR /home/$USER

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]




