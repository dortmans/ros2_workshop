# ROS2 Distributed

ROS 2 uses DDS for communication between nodes. It allows you to run nodes on different physical machines as long as they are in the same LAN network (with multicasting enabled). To run ROS on robotic systems distributed over multiple networks (connected via Internet) secure routing is needed.

In general:

- DDS is not a good match for (unreliable) Wifi networks
- DDS is not scalable over the Internet

In particular following problems need to be solved:

- Discovery overhead
- Routing over Internet

Following options are available:

- eProsima [DDS Router](https://eprosima.com/index.php/products-all/eprosima-dds-router) & [Discovery Server](https://eprosima.com/index.php/products-all/tools/eprosima-discovery-server)
- Zenoh [DDS bridge](https://github.com/eclipse-zenoh/zenoh-plugin-dds) and [Zenoh Router](https://zenoh.io/docs/getting-started/installation/)
- Husarnet [Peer-to-Peer VPN](https://husarnet.com/docs/tutorial-ros2)

## References

[16.2. Use ROS 2 with Fast-DDS Discovery Server](https://fast-dds.docs.eprosima.com/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html)

[eProsima DDS Router Documentation](https://eprosima-dds-router.readthedocs.io/en/latest/)

[Using Husarnet & ROS 2](https://husarnet.com/docs/tutorial-ros2/)

[Connecting Remote Robots Using ROS2, Docker & VPN](https://husarnet.com/blog/ros2-docker)

[Scalable Distributed Robot Fleet With Fast DDS Discovery Server](https://husarnet.com/blog/ros2-dds-discovery-server/)

[Bridge Remote DDS Networks With a DDS Router](https://husarnet.com/blog/ros2-dds-router/)

[Integrating ROS2 with Eclipse zenoh](https://zenoh.io/blog/2021-04-28-ros2-integration/)

[Minimizing Discovery Overhead in ROS2](https://zenoh.io/blog/2021-03-23-discovery/)