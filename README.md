# Bridging
This branch will focus on bridging from ROS1 to ROS2. This is necessary because RMF runs on ROS2. Thus the vendor navstack will need to expose certain topics to ROS2. We will use [ros1_bridge](https://github.com/ros2/ros1_bridge) for this purpose. 

`sudo apt install ros-dashing-ros1-bridge`

We also restructure the workspace in order to separate the ros1 and ros2 workspaces.

The important topics are identified as:
* `/move_base_simple/goal`: The fleet manager will instruct the robot on target destinations by publishing `geometry/PoseStamped` messages on this topic.
* `move_base_node/SBPLLatticePlanner/plan` : The robot navstack will publish a `nav_msgs/Path` message which describes the path to take, which the fleet manager will translate for the RMF scheduler.


## Testing the ROS Bridge
* In one terminal: `cd ros1 && source install/setup.bash && roslaunch mir_vendor_setup main.launch` to run the MiR simulation from master branch. **Remember** to start the physics in the simulation after everything is online.
* In another terminal, after sourcing ROS1 and then ROS2: `cd ros2 && source install/setup.bash && ros2 run ros1_bridge dynamic_bridge` to run the bridging node.
* From yet another ROS2 terminal, publish a nav goal as a ROS2 PoseStamped message:
```
ros2 topic pub /move_base_simple/goal geotry_msgs/PoseStamped '
{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, 
pose: {position: {x: 17.0, y: 10.0, z: 0.0}, 
orientation: {w: 1.0}}}'
```

The gazebo simulation should receive the nav goals as if it was published in ROS1.

