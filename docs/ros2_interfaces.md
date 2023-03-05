# ROS2 interfaces

## ROS2 interface concepts

ROS2 supports three node [interface concepts](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/):

- [topics](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/)
- [services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)
- [actions](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/)

Under the hood ROS2 uses the DDS (Data Distribution Service) protocol to implement these node interfaces.

Topics are meant to be used for continuous data streams (e.g. sensor data, robot state).

A topic is defined by:

- the name of the topic (e.g. cmd_vel)
- a .msg file containing the definition of a message (e.g. [TwistStamped.msg](https://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html))

Services are meant for remote procedure calls that terminate quickly, e.g. for querying or setting the state of a node or for calling a quick calculation. A service call is blocking the caller.

A service is defined by:

- the name of the service (e.g. enable_motor)
- a .srv file containing the definition of a request and a respons message (e.g. [SetBool.srv](https://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))

Actions should be used for activating long running activities such as e.g. moving a robot to goal pose. An action can be preempted, i.e. stopped before completion.

An action is defined by:

- the name of the action (e.g. navigate_to_pose)
- a .action file containing the definition of a goal, a result, and a feedback message (e.g. [NavigateToPose.action](https://github.com/ros-planning/navigation2/blob/main/nav2_msgs/action/NavigateToPose.action))

Note that a lot of [common interfaces](https://github.com/ros2/common_interfaces) have already been defined. *Reusing these common interface definitions should be preferred over creating your own proprietary message, service or action definitions.*

## Create an interface package

Create a package dedicated to ROS2 custom messages

Specify package and workspace:
```
PACKAGE=my_msgs
STACK=my_stack
DEPENDENCIES="builtin_interfaces std_msgs geometry_msgs nav_msgs diagnostic_msgs action_msgs"
WORKSPACE=~/ros2_ws
```

Now create the new ROS2 package:
```
cd ${WORKSPACE}/src/${STACK}
ros2 pkg create ${PACKAGE} --build-type ament_cmake --dependencies ${DEPENDENCIES}
cd ${PACKAGE}
rm -rf include/ src/
mkdir msg srv action
```

Take care that git keeps empty directories:
```
touch msg/.gitkeep
touch srv/.gitkeep
touch action/.gitkeep
``` 

Put your custom interface definitions in the msg, srv and action directories.
Let us add an empty message, service and action to be able to test the build process.
```
touch msg/Empty.msg
echo -e "# Request\n---\n# Result" > srv/Empty.srv
echo -e "# Request\n---\n# Result\n---\n# Feedback" > action/Empty.action
```

Open package.xml file and add following lines:
```
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Open the CMakeLists.txt file and add following lines:

```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
#	your custom interfaces will be here
#	one per line
#	no comma for separating lines
	"msg/Empty.msg"
	"srv/Empty.srv"
	"action/Empty.action"
) 
```

Build your interface package

```
cd ${WORKSPACE}
colcon build --packages-up-to ${PACKAGE}
```

## How to find installed interfaces

List of installed interfaces:
```
ros2 interface list
```

Find custom interfaces:
```
ros2 interface list | grep ${PACKAGE}
```

## References

[About ROS 2 interfaces](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/)

[Understanding ROS 2 nodes](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Nodes/)

[Understanding ROS 2 topics](https://index.ros.org/doc/ros2/Tutorials/Topics/Understanding-ROS2-Topics/)

[Understanding ROS 2 services](https://index.ros.org/doc/ros2/Tutorials/Services/Understanding-ROS2-Services/)

[Understanding ROS 2 actions](https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/)

[Topics vs Services vs Actions](https://docs.ros.org/en/rolling/How-To-Guides/Topics-Services-Actions.html)

[ros2/common_interfaces](https://github.com/ros2/common_interfaces)

[ROS2 Create Custom Message (Msg/Srv)](https://roboticsbackend.com/ros2-create-custom-message/)

[Creating custom msg and srv files](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

[Implementing custom interfaces](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html)

[Creating an action](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Creating-an-Action.html)







