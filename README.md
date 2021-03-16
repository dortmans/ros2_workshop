# ros2_workshop

Get going with ROS2.

## ROS2 installation

Install ROS 2 Foxy Fitzroy following the [instructions on the wiki](https://index.ros.org/doc/ros2/Installation/Foxy/).
This [ROS2 setup script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu) will do the work for you.

## ROS2 via Docker

Instead of installing ROS2 you can also opt for using Docker.

To install Docker on Ubuntu follow instructions [here](https://docs.docker.com/engine/install/ubuntu/).

Step into 'docker' folder of this repository:
```
cd docker
```

Build a ROS2 docker image as follows:
```
./build_ros2.sh
```
This command only has to be run once (or each time after making changes to the Dockerfile).
You can modify the Dockerfile to install additional packages.

To run a ROS2 shell open a terminal and enter following command:
```
./run_ros2.sh
```

You can also directly execute a command, e.g.:
```
./run_ros2.sh ros2 topic list
```
or
```
./run_ros2.sh rviz2
```

Alternatively to run a complete ROS2 desktop in your browser enter following command in a terminal window:
```
./ros2_desktop.bash
```
Then open `http://localhost` in your browser to access the desktop.
Use the 'Clipboard' to copy/paste commands.

If you want to execute the scripts mentioned on this page from any directory add the following line to your '.bashrc' file:
```
export PATH=$PATH:$HOME/ros2_workshop/docker
```

You can do this by using an editor or by running following command:
```
echo 'export PATH=$PATH:$HOME/ros2_workshop/docker' >> $HOME/.bashrc
```

Note: The build script should always be run in the ros2_workshop/docker directory.

