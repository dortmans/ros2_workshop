# ros2_workshop

Get going with ROS2.

## ROS2 installation

Install ROS 2 Foxy Fitzroy following the [instructions on the wiki](https://index.ros.org/doc/ros2/Installation/Foxy/)

## ROS2 via Docker

Instead of installing ROS2 you can also opt for using Docker.

To install Docker on Ubuntu follow instructions [here](https://docs.docker.com/engine/install/ubuntu/).

Step into 'docker' folder:
```
cd docker
```

Build a ROS2 docker image as follows:
```
./build.sh
```
This command only has to be run once (or each time after making changes to the Dockerfile).
If you want you can modify the Dockerfile to install additional packages.

To run a ROS2 shell open a terminal and enter following command:
```
./run.sh
```

You can also directly execute a command, e.g.:
```
./run.sh ros2 topic list
```
or
```
./run.sh rviz2
```

Alternatively to run a complete ROS2 desktop in your browser enter following command in a terminal window:
```
./ros2_desktop.bash
```
Then open `http://localhost` in your browser to access the desktop.
Use the 'Clipboard' to copy/paste commands.


