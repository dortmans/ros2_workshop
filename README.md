# ros2_workshop

Get going with ROS2.

## ROS2 installation

Install ROS 2 Foxy Fitzroy following the [instructions on the wiki](https://index.ros.org/doc/ros2/Installation/Foxy/).
This [ROS2 setup script](https://github.com/Tiryoh/ros2_setup_scripts_ubuntu) can do the work for you.

## ROS2 via Docker

Instead of installing ROS2 on your computer you can also opt for using Docker.

To install Docker on Ubuntu follow instructions [here](https://docs.docker.com/engine/install/ubuntu/). On Ubuntu it is as simple as running following command:
```
curl https://get.docker.com | sh && sudo systemctl --now enable docker
```

Do not forget to [add your user to the docker group](https://docs.docker.com/engine/install/linux-postinstall/), because we need you to run docker without sudo!
```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Log out and log back in so that your group membership is re-evaluated. 
Now you should be able to run Docker without sudo.

If you have an NVIDIA GPU, install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker):
```
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

Clone the ros2_workshop repository:
```
git clone https://github.com/dortmans/ros2_workshop.git
```

Step into the 'docker' folder of this repository and build a ROS2 docker image as follows::
```
cd docker
./build_ros2.sh
```

You can modify the Dockerfile to install additional packages.

The build script only has to be run once or each time after making changes to the Dockerfile.
It should always be run in the directory where the Dockerfile is. 

To execute the other scripts mentioned on this page from any directory add the following line to your '.bashrc' file:
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

If you get a docker error message like `could not select device driver .. with capabilities: [[gpu]]` you do not have a working GPU available. Maybe you are running in a Virtual Machine?
In this case open the start_ros2.sh script in your editor and remove following line: `--gpus all \`

Whenever you want to stop the ROS2 container in the background, just open a terminal window and run the stop script:
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
