# ROS2 Parameters

Example YAML config file with wildcard matching all nodes in all namespaces (/**):

```
/**:
  ros__parameters:
    int_number: 42

ns1:
  node1:
    ros__parameters:
      float_param: 0.1

node2:
  ros__parameters:
    float_param: 45.2
```

Dump all parameters of a node:

```
ros2 param dump <node-name> --print
```

[rclcpp Params Tutorial – Get and Set ROS2 Params with Cpp](https://roboticsbackend.com/rclcpp-params-tutorial-get-set-ros2-params-with-cpp/)

[rclpy Params Tutorial – Get and Set ROS2 Params with Python](https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/)

[ROS2 YAML For Parameters](https://roboticsbackend.com/ros2-yaml-params/)

[ROS2 Global Parameters HowTo](https://roboticsbackend.com/ros2-global-parameters/)

[ROS2 rclpy Parameter Callback [Tutorial]](https://roboticsbackend.com/ros2-rclpy-parameter-callback/)


