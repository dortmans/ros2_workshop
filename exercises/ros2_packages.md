# ROS2 packages

## Create a package

There are several different packages one can create:
- [A C++ package](https://roboticsbackend.com/create-a-ros2-cpp-package/)
- [A Python package](https://roboticsbackend.com/create-a-ros2-python-package/)
- [A package for both Python and C++](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
- [A package dedicated to ROS2 custom messages](https://roboticsbackend.com/ros2-create-custom-message/)

Here we create a package for both C++ and Python nodes.

Specify package and workspace:
```
PACKAGE=my_package_name
STACK=my_stack_name
DEPENDENCIES="rclcpp rclpy"
WORKSPACE=~/ros2_ws
```

Now create the new ROS2 package:
```
cd ${WORKSPACE}/src/${STACK}
ros2 pkg create ${PACKAGE} --build-type ament_cmake --dependencies ${DEPENDENCIES}
cd ${PACKAGE}
mkdir ${PACKAGE}
touch ${PACKAGE}/__init__.py
mkdir scripts
mkdir launch
mkdir config
cd ${WORKSPACE}
sudo apt update
#rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO
colcon build --symlink-install --packages-up-to ${PACKAGE}
```

Edit the package.xml file and add following line:
```
<buildtool_depend>ament_cmake_python</buildtool_depend>
```

Edit the CMakeLists.txt file.

Remove following lines:
```
# Default to C99
if(NOT CMAKE_C_STANDARD)echo m
  set(CMAKE_C_STANDARD 99)
endif()
```

Add following line under '# Find dependencies':
```
find_package(ament_cmake_python REQUIRED)
```

After the find_package lines add lines for compiling and installing your Cpp and Python executables:
```
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_executable
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (and other scripts)
install(PROGRAMS
  scripts/py_node.py
  scripts/script.sh
  DESTINATION lib/${PROJECT_NAME}
)

```

To install your launch files you’ll need to add this to your CMakeLists.txt:

```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
```

The same for your config files:
```
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
```

## How to find installed packages

List of installed packages:
```
ros2 pkg list
```

Full path to installed package:
```
ros2 pkg prefix --share <package_name>
```

Example to show a list of all launch files of package turtlesim:
```
ls $(ros2 pkg prefix --share turtlesim)/launch
```

## References

[Create a ROS2 package for Both Python and Cpp Nodes - The Robotics Back-End](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)

[Create a ROS2 Cpp Package](https://roboticsbackend.com/create-a-ros2-cpp-package/)

[Create a ROS2 Python package](https://roboticsbackend.com/create-a-ros2-python-package/)

[ROS2 Create Custom Message (Msg/Srv)](https://roboticsbackend.com/ros2-create-custom-message/)

[Creating your first ROS 2 package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/)

[ROS2 Tutorials #4: How to create a ROS2 Package for C++](https://www.theconstructsim.com/ros2-tutorials-create-a-ros2-package-cpp/)

[ament_cmake user documentation](https://docs.ros.org/en/foxy/Guides/Ament-CMake-Documentation.html)

