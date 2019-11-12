# Fleet Manager
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b 01-fleet-manager`

This chapter will focus on the fleet manager. The role of the fleet manager is to process high level commands, into low level actions that can be carried out by the robot. In some cases, certain vendors will already have written fleet managers for their systems. In our example, this is not the case, so we build a simple one. Recall that our MiR takes in `geometry_msgs/PoseStamped` messages as nav goals. The role of our fleet manager is to manage the delivery, modification and cancellation of such commands, especially in the case of multiple message submissions. In particular, the fleet manager will also expose an interface to ROS2 via bridging in order to communicate with the fleet adapters.

## Building the Fleet Manager
<!-- 
This branch will focus on bridging from ROS1 to ROS2. This is necessary because RMF runs on ROS2. Thus the vendor navstack will need to expose certain topics to ROS2. We will use [ros1_bridge](https://github.com/ros2/ros1_bridge) for this purpose. 

`sudo apt install ros-dashing-ros1-bridge`

We also restructure the workspace in order to separate the ros1 and ros2 workspaces.

The important topics are identified as:
* `/move_base_simple/goal`: The high level systems in ROS2 will instruct the robot on target destinations by publishing `geometry/PoseStamped` messages on this topic.
* `/move_base_node/SBPLLatticePlanner/plan` : The lower level robot navstack in ROS1 will publish a `nav_msgs/Path` message which describes the path it will take. This information can be used by the high lvel systems for the RMF scheduler.

## Building the packages
`cd` into each of `ros1` and `ros2` and do a `colcon build`, sourcing the respective ROS environments.

## Testing the ROS Bridge
In the following, a ROS1 terminal is a terminal sourced with ROS1 and sourced with the corresponding `ros1` workspace in the tutorial, vice versa for ROS2.

* In one ROS1 terminal (1): `roslaunch mir_vendor_setup main.launch` to run the MiR simulation from master branch. **Remember** to start the physics in the simulation after everything is online.
* In a ROS1 & ROS2 terminal (2): `ros2 run ros1_bridge dynamic_bridge` to run the bridging node.
* From a ROS2 terminal (3), run the node that will listen for `/move_base_node/SBPLLatticePlanner/plan` messages over the bridge: `ros2 run fleet_ros2_bridge bridge_node`.
* From another ROS2 terminal (4), publish a nav goal as a ROS2 PoseStamped message:
```
ros2 topic pub /move_base_simple/goal geotry_msgs/PoseStamped '
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, 
pose: {position: {x: 17.0, y: 10.0, z: 0.0}, 
orientation: {w: 1.0}}}'
```

The gazebo simulation should receive the ROS2 nav goals as if it was published in ROS1. Additionally, we should see the ROS1 plan messages as if it was published in ROS2.
 -->
