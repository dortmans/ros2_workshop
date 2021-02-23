FROM ros:foxy 

ENV DEBIAN_FRONTEND=noninteractive

# Install extra packages
RUN apt-get update && apt-get install -y \
		libgl1-mesa-glx libgl1-mesa-dri \
    mesa-utils mesa-utils-extra \
    iputils-ping \
    ros-$ROS_DISTRO-desktop \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# nvidia-docker2 settings
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

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
