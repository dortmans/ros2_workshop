# ROS2 getting started

Setup
```
source /opt/ros/$ROS_DISTRO/setup.bash
```

Check environment variables:
```
env | grep ROS
echo $ROS_DISTRO
```	

Publish on a topic:
```
ros2 topic pub /chatter std_msgs/String "data: Hello World"
```

Subscribe to this topic:
```
ros2 topic echo /chatter
```

Start a node that simulates a turtle:
```
ros2 run turtlesim turtlesim_node
```

List running nodes, their parameters, topics, services and actions:
```
ros2 node list
ros2 param list
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

Publish a Twist message on the /turtle1/cmd_vel topic to make turtle1 move:
```
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Send an action goal to make turtle1 rotate:
```
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.14}"
```

Call a service to spawn another turtle:
```
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.0, name: 'tortise'}"
```

Run the turtle_teleop_key node to drive the turtle1 around using your keyboard:
```	
ros2 run turtlesim turtle_teleop_key
```

In another terminal window start rqt_graph to show the ROS2 computation graph and see all running nodes are connected:
```
rqt_graph
```

## References

[Getting Started with ROS 2](https://discourse.ubuntu.com/t/getting-started-with-ros-2/17847)

	 
	

	

	




