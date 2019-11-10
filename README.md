# Waypoint Mapping
We initially assume that the vendor setup comes with a complete navigation stack. This includes a navigation map. We can find the map for this example in `ros1/src/mir_robot/mir_gazebo/maps/maze.png`. The next step would be to annotate this map with waypoints and lanes, which represent the "traffic roads" on which our robot(s) will operate. 

Note that this step is done once per "area of operation", regardless of the number of fleets operating. ( IE O(1) with respect to n fleet adapters ). We will use [rmf-editor](https://github.com/osrf/rmf-editor.git) for this job. We also refactor the directory space for external modules, which are not ROS packages, by adding the `external` folder.

## Initialization
* `sudo apt update`
* `sudo apt install git cmake libyaml-cpp-dev qt5-default`
* `cd external/src && mkdir build`
* `cmake ..`
* `make -j4`

This should be sufficient to build the `rmf-editor` locally in the `external` folder. The executable is found in `/external/build/src/rmf-editor/rmf-editor`.

