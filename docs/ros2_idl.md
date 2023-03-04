# ROS2 IDL Mapping

ROS 2 supports a [subset](https://design.ros2.org/articles/idl_interface_definition.html) of the [OMG IDL 4.2 specification](https://www.omg.org/spec/IDL/4.2). 

ROS2 translates 'msg', 'srv' and 'action' file into 'idl' files. These idl files are then translated into C++ code.
IDL files from native DDS applications can in most cases be used by ROS2 to communicate with those applications.
They first must be manually cleansed however.

Split each idl file in separate idl files, each containing only one struct. Store the idl files in your `package_name/msg` directory. Give each file the name of the struct that it contains, e.g. 'MessageName.idl'. 

Each ROS2 compatible idl file should have following structure:
```
module package_name{
	module msg {
	
		struct MessageName {
			....
		};
		
	};
};
```

Remove unsupported elements like #comments and enums. Enums can be replaced by a module with constant definitions like e.g.
```
module AlarmLevel {
	const uint32 WARNING = 0;
	const uint32 ERROR = 1;
	const uint32 CRITICAL = 2;
	const uint32 EMERGENCY = 3;
};
```

Prefix each typename with `package_name::::msg::`, e.g. `RobotPose robotPose;` becomes `package_name::msg::RobotPose robotPose;` and `sequence<AlarmAttribute> attributes;` becomes `sequence<package_name::msg::AlarmAttribute> attributes;`.

In your `CMakeLists.txt` file add each idl file to `rosidl_generate_interfaces`:
```
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/MessageName.idl"
	"msg/OtherMessageName.idl"
	"...
) 
```

## References

[ROS 2 Design - IDL - Interface Definition and Language Mapping](https://design.ros2.org/articles/idl_interface_definition.html)

