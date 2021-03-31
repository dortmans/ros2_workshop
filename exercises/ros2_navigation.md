# ROS2 Navigation

Install dependencies
```
sudo apt install wget python3-vcstool
```

Install Nav2:
```
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
```

Install TurtleBot3 Packages
```
# sudo apt install '~ros-foxy-turtlebot3-.*'
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3.repos
vcs import src < turtlebot3.repos
rosdep update && rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
colcon build --symlink-install
```

Optionally put following lines in your ~/.bashrc file
```
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
echo 'export TURTLEBOT3_MODEL=waffle' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

Run navigation demo:
```
export TURTLEBOT3_MODEL=waffle
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py
```

Make sure you set the initial pose by clicking 2D Pose Estimate button in RViz2 or by executing the following command:
```
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
    "header": { "frame_id": "map" },
    "pose": {
        "pose": {
            "position": { "x": 0.0, "y": 0.0, "z": 0.0 },
            "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 }
        }
    }
}'
```

Move your robot by requesting a goal using RViz 2D Nav goal button or using the ROS2 CLI, e.g.
```
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### Alternative approach:

Launch simulated world
```
export TURTLEBOT3_MODEL='waffle'
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Start nav2:
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
```

## SLAM

SLAM Toolbox can be installed via:
```
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

```
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=true
```

## References

[Simulate the TurtleBot3](https://ubuntu.com/blog/simulate-the-turtlebot3)

[Robotis TurtleBot3 eManual - Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)

[Nav2 Getting Started](https://navigation.ros.org/getting_started/index.html)

[Navigating with a Physical Turtlebot 3](https://navigation.ros.org/tutorials/docs/navigation2_on_real_turtlebot3.html)

[(SLAM) Navigating While Mapping](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)

[cyberbotics/webots_ros2 - Navigate TurtleBot3](https://github.com/cyberbotics/webots_ros2/wiki/Navigate-TurtleBot3)

[cyberbotics/webots_ros2 - SLAM with TurtleBot3](https://github.com/cyberbotics/webots_ros2/wiki/SLAM-with-TurtleBot3)

[A ROS2 Nav2 navigation tf2 tutorial using turtlesim](https://blog.hadabot.com/ros2-navigation-tf2-tutorial-using-turtlesim.html)
- [Run the Navigation2, tf2, turtlesim example](https://github.com/hadabot/hadabot_main/tree/master/content/p8)

[ROS2 Nav2 Go-To-Goal using the low-cost Hadabot Turtle robot kit](https://blog.hadabot.com/ros2-nav2-go-to-goal-low-cost-robot-kit.html)
- [Run the Navigation2 Go-To-Goal example with the Hadabot Turtle robot](https://github.com/hadabot/hadabot_main/tree/master/content/p9)

[The Marathon 2: A Navigation System](https://arxiv.org/abs/2003.00368)

