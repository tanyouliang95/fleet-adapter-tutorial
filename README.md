# Fleet Adapter Tutorial
This is an attempt at documenting the development of a RMF fleet adapter. We start with a [MiR robot](https://github.com/dfki-ric/mir_robot) in simulation, and document the steps required to integrate this "vendor ready" system with RMF. 

We will record the steps of this tutorial as branches. The first branch, `master`, documents the provision of a "vendor ready" system (MiR). Each subsequent branch will record each discrete step required to integrate with RMF in the corresponding branches README.md.

## Changing Chapters
It is best to delete `fleet-adapter-tutorial` and run `git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b [branch]`, in order to present dangling untracked files.

## Setting up the MiR simulation
The following commands will set up a simulation MiR with navstack, in a maze environment. We can consider that mapping has already been done. Remember to manually unpause the simulation once the stack finishes loading.

* `git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b master`
  * Do the following just once to install dependencies:
  * `cd src/mir_robot`
  * `sudo apt-get update`
  * `sudo apt-get install -y python-rosdep`
  * `sudo rosdep init`
  * `rosdep update`
  * `rosdep install --from-paths ./ -i -y --rosdistro melodic`
* `cd ../.. && colcon build`
* `source install/setup.bash`
* `roslaunch mir_vendor_setup main.launch`
* Unpause the physics simulation in Gazebo ( press the play button )

The MiR receives goal poses by subscribing to geometry_msgs/PoseStamped published to `/move_base_simple/goal`. Instead of clicking in Rviz, we can alternatively publish a PoseStamped message, such as the one below:
```
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{ header: { frame_id:  "map"}, 
pose: { position: { x: 17, y: 10 }, 
orientation: { x: 0, y: 0, z: 0, w: 1 } } }'
```
