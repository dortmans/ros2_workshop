# ros2_workshop

Get going with ROS2.

## ROS2 installation

Install ROS 2 Foxy Fitzroy following the [instructions on the wiki](https://index.ros.org/doc/ros2/Installation/Foxy/).
This [ROS2 setup script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu) can do the work for you.

## ROS2 via Docker

Instead of installing ROS2 on your computer you can also opt for using Docker.

To install Docker on Ubuntu follow instructions [here](https://docs.docker.com/engine/install/ubuntu/).

Step into the 'docker' folder of this repository and build a ROS2 docker image as follows::
```
cd docker
./build_ros2.sh
```

You can modify the Dockerfile to install additional packages.
The build script only has to be run once or each time after making changes to the Dockerfile.
It should always be run in the directory where the Dockerfile is. To execute the other scripts mentioned on this page from any directory add the following line to your '.bashrc' file:
```
export PATH=$PATH:$HOME/ros2_workshop/docker
```

You can do this by using an editor or by running following command:
```
echo 'export PATH=$PATH:$HOME/ros2_workshop/docker' >> $HOME/.bashrc
```

To start a ROS2 session open a terminal and enter following command:
```
start_ros2.sh
```
This will start a ROS2 container in the background which keeps running even when you close the terminal.
Whenever you want to stop it just open a terminal window and run the stop script:
```
stop_ros2.sh
```

While the ROS2 container is running in the background you can open new  interactive shells in this container. Anytime you want to open a new ROS2 shell you just have to open a new terminal window and enter following command:
```
run_ros2.sh
```

You can also directly execute a ROS2 command, e.g.:
```
run_ros2.sh ros2 topic list
```

It even works with even a graphical application like rviz2:
```
run_ros2.sh rviz2
```

## ROS2 in your browser

Alternatively to run a complete ROS2 desktop in your browser (based on docker plus [noVNC](https://novnc.com) technology) enter following command in a terminal window:
```
ros2_desktop.bash
```
Then open `http://localhost` in your browser to access the desktop.
Use the 'Clipboard' to copy/paste commands.
