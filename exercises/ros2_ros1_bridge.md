# ROS1 bridge

The prebuilt ROS 2 binaries includes support for common ROS interfaces (messages/services), such as the interface packages listed in the [ros2/common_interfaces](https://github.com/ros2/common_interfaces) repository and [tf2_msgs](https://github.com/ros2/geometry2/tree/ros2/tf2_msgs).
Built-in message and service types are automatically bridged, with no additional work required. All that is required is to run the bridge.

Custom message and service types require recompilation of the bridge!

## Installation

Before installing ros1_bridge you first need to add the ROS 1 APT repository to your sources.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Now you can install the bridge:
```
sudo apt update
sudo apt install ros-foxy-ros1-bridge
```

## Usage

You first have to setup both ROS1 and ROS2 in the following exact sequence:
```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
```

See all built-in types automatically bridged:
```
ros2 run ros1_bridge dynamic_bridge --print-pairs
```

Run the bridge:
```
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

[ros1_bridge](https://github.com/ros2/ros1_bridge)

[ROS1-ROS2 Bridge Demo](https://industrial-training-master.readthedocs.io/en/melodic/_source/session7/ROS1-ROS2-bridge.html)

[mabelzhang/ros1_bridge_sandbox](https://github.com/mabelzhang/ros1_bridge_sandbox)

[ROS2 Tutorials #8: How to communicate between ROS1 & ROS2 with ros1_bridge](https://www.theconstructsim.com/how-to-communicate-between-ros1-ros2-with-ros1_bridge/)

[Testing ROS Bridge on the TurtleBot3](https://www.slblabs.com/2020/08/04/testing-ros-bridge-on-the-turtlebot3/)

[ROS Answers - Install Ros1-Bridge on Foxy is Failing](https://answers.ros.org/question/354207/install-ros1-bridge-on-foxy-is-failing/)
