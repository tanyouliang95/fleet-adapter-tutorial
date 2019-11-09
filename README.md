# Bridging
This branch will focus on bridging from ROS1 to ROS2. This is necessary because RMF runs on ROS2. Thus the vendor navstack will need to expose certain topics to ROS2. We will use [ros1_bridge](https://github.com/ros2/ros1_bridge) for this purpose. 

`sudo apt install ros-dashing-ros1-bridge`

We also restructure the workspace in order to separate the ros1 and ros2 workspaces.


