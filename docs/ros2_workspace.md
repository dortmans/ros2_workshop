# ROS2 Workspace

Create a workspace `ros2_ws` using the following command:
```
mkdir -p ~/ros2_ws/src
```
Optionally clone one or more repositories within the `src` folder:
```
cd ~/ros2_ws/src
git clone https://github.com/ros2/examples -b $ROS_DISTRO
```

Finally, change directory back to the top of your workspace and build using `colcon`:
```
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install
```

In the above example we have used `ros2_ws` as workspace name but you can use any other name you want. 

Often ROS developers have several differently named workspaces for different projects. Always before use you have to source the setup script of the workspace you are going to use. In this case:
```
source ~/ros2_ws/install/setup.bash
```

If you have just one (mostly used) workspace it is common practice to add this source command at the end of your .bashrc file so that it is automatically executed in each terminal window you open.