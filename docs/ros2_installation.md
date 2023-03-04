# Install ROS2

[This](https://index.ros.org/doc/ros2/Installation/Foxy/) link will direct you to the installation instructions for the latest version of ROS2 (Foxy). Please follow the instructions and install the right version for your system.

[This ROS2 setup script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu) or [this one](https://github.com/ROBOTIS-GIT/robotis_tools/blob/master/install_ros2_foxy.sh) will do the work for you.


### Change RMW Implementation (Optional)

Cyclone DDS will be the standard DDS implementation in newer ROS2 releases. It is an optional change to the installation in ROS2 Foxy.

Install CycloneDDS:
```
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```

To make ROS2 use this particular DDS run the following command in each terminal window you open:

```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If you want to make this more permanent, run the following command which will add it to your `.bashrc` file. Make sure to only run this once!

```
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

