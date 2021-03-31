# ROS2 interfaces

## Create an interface package

Create a package dedicated to ROS2 custom messages

Specify package and workspace:
```
PACKAGE=my_msgs
STACK=my_stack
DEPENDENCIES="builtin_interfaces action_msgs"
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

Find our custom interfaces:
```
ros2 interface list | grep ${PACKAGE}
```

## References

[About ROS 2 interfaces](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)

[ROS2 Create Custom Message (Msg/Srv)](https://roboticsbackend.com/ros2-create-custom-message/)

[Creating custom ROS 2 msg and srv files](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)

[Creating an action](https://docs.ros.org/en/foxy/Tutorials/Actions/Creating-an-Action.html)

[Expanding on ROS 2 interfaces](https://docs.ros.org/en/foxy/Tutorials/Single-Package-Define-And-Use-Interface.html)
