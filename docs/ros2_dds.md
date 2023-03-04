# ROS2 DDS

ROS2 uses [DDS](https://en.wikipedia.org/wiki/Data_Distribution_Service) as middleware (RMW: ROS Middleware) for communication between nodes. It is compatible with multiple DDS implementations.

FastDDS (FastRTPS) and CycloneDDS currently qualify as Tier 1 RMW implementations. FastDDS is the default in the ROS 2 Foxy release. CycloneDDS will become the default in ROS 2 Galactic. 

## Installation and configuration of CycloneDDS

To check which RMW you are currently using:
```
ros2 doctor --report | grep middleware
```

[Working with Eclipse Cyclone DDS](https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html) explains how to install and utilize Cyclone DDS.

To install it:
```
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
```

The environment variable `RMW_IMPLEMENTATION` determines which RMW (ROS MiddleWare) implementation will be used. To switch to Cyclone DDS:
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

If you have multiple network interfaces (e.g. Ethernet and Wifi) you should also select the network interface that Cyclone DDS should use:
```
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>YOUR_NETWORK_INTERFACE</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
```

Use following command to find the name of your Wifi interface:
```
iw dev | awk '$1=="Interface"{print $2}'
```

Note: To install iw run `sudo apt install iw`.

## Domain ID

In DDS, the primary mechanism for having different logical networks share a physical network is known as the [DOMAIN ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html). ROS 2 nodes on the same domain can freely discover and send messages to each other, while ROS 2 nodes on different domains cannot. To avoid interference between different groups of computers running ROS 2 on the same network, a different domain ID should be set for each group.

DDS uses UDP multicast to communicate the metadata allowing different node to discover each other and establish communication. All the machines that will be talking to each other must satisfy two constraints. First, they must all be within the same multicast domain on your network. Secondly, then they must have the same ROS_DOMAIN_ID. 

All ROS 2 nodes use domain ID 0 by default.
So, if you don’t want to interfere with other DDS systems (ROS or non-ROS) on the same network or you want to join a particular DDS system you have to set the `ROS_DOMAIN_ID` environment variable, e.g.:
```
export ROS_DOMAIN_ID=42
```

Note: You can add all above described export commands to your .bashrc file if you want them to be automatically executed in each terminal you open.

## Quality of Service

ROS 2 offers a rich variety of Quality of Service (QoS) policies that allow you to tune communication between nodes. 

To configure QoS properly in Python, for example in a subscriber, you can use the following:

```
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

qos_policy = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=1)
self.sub = self.create_subscription(Image, topic, self._callback, qos_profile=qos_policy)
```

You can use the `ros2 topic info /topic_name --verbose` command to view the QoS configuration of the Publishers and Subscribers of that topic.

## ROS2 topic name mangling

ROS2 uses the following mangled topics when the ROS QoS policy `avoid_ros_namespace_conventions` is false, which is the default:

- Topics are prefixed with rt. e.g.: /my/fully/qualified/ros/topic is converted to rt/my/fully/qualified/ros/topic.
- The service request topics are prefixed with rq and suffixed with Request. e.g.: /my/fully/qualified/ros/service request topic is rq/my/fully/qualified/ros/serviceRequest.
- The service response topics are prefixed with rr and suffixed with Response. e.g.: /my/fully/qualified/ros/service response topic is rr/my/fully/qualified/ros/serviceResponse.

Since all ROS topics are prefixed when being converted to DDS topic names it makes it impossible to subscribe to existing DDS topics which do not follow the same naming pattern.

If `avoid_ros_namespace_conventions` is true, any ROS specific namespacing conventions will be circumvented.
This might be useful when trying to directly connect a native DDS topic with a ROS 2 topic.
However, making it true removes the rt prefix but does not prevent ROS2 Foxy to prepend a / to the topic name.

For instance run following code:
```
qos = rclpy.qos.QoSProfile(avoid_ros_namespace_conventions=True, depth=10)
self._sub1 = self.create_subscription(String, "my_raw_topic_sub", handler, qos)
self._pub1 = self.create_publisher(String, "my_raw_topic_pub", qos)
self._sub2 = self.create_subscription(String, "my_topic_sub", handler, 10)
```
It will create topics: /my_raw_topic_sub, /my_raw_topic_pub and rt/my_topic_sub.

## References

[Data Distribution Service](https://en.wikipedia.org/wiki/Data_Distribution_Service)

[ROS 2 middleware interface](https://design.ros2.org/articles/qos.html)

[Installing DDS implementations](https://docs.ros.org/en/foxy/Installation/DDS-Implementations.html)

[Working with Eclipse Cyclone DDS](https://docs.ros.org/en/foxy/Installation/DDS-Implementations/Working-with-Eclipse-CycloneDDS.html)

[Some Notes on Cyclone DDS](http://www.robotandchisel.com/2020/08/12/cyclonedds/)

[The ROS_DOMAIN_ID](https://docs.ros.org/en/foxy/Concepts/About-Domain-ID.html)

[Topic and Service name mapping to DDS](https://design.ros2.org/articles/topic_and_service_names.html)

[About Quality of Service settings](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)

[Quality of Service Demos](https://github.com/ros2/demos/tree/master/quality_of_service_demo)

[ROS2 Cookbook - Networking](https://github.com/mikeferguson/ros2_cookbook/blob/main/pages/networking.md)

