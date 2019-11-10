# Waypoint Mapping
We initially assume that the vendor setup comes with a complete navigation stack. This includes a navigation map. We can find the map for this example in `ros1/src/mir_robot/mir_gazebo/maps/maze.png`. The next step would be to annotate this map with waypoints and lanes, which represent the "traffic roads" on which our robot(s) will operate. 

Note that this step is done once per "area of operation", regardless of the number of fleets operating. ( IE O(1) with respect to n fleet adapters ). We will use [traffic-editor](https://github.com/osrf/traffic-editor.git) for this job. We also refactor the directory space for external modules, which are not ROS packages, by adding the `external` folder.

## Building traffic-editor
* `sudo apt update`
* `sudo apt install git cmake libyaml-cpp-dev qt5-default`
* `cd external/src && mkdir build && cd build`
* `cmake ..`
* `make -j4`

This should be sufficient to build the `traffic-editor` locally in the `external` folder. The executable is found in `/external/build/src/traffic-editor/traffic-editor`.

## Running traffic-editor
The following steps may be necessary until `traffic-editor` is more developed.
* `cd external/src/traffic-editor`. We need to run the executable from the `traffic-editor` source folder.
* `../../build/src/traffic-editor/traffic-editor`. Run the executable.
* Select file -> Open Project, and select `maze.yaml` from `maps/maze-new` or `maps/maze-complete`.

**Important** to note that the executable should be run from `traffic-editor` source folder, and the path specified in `maze.yaml` must point correctly to `maze.png` relative to the `traffic-editor` source folder.

## Introduction
The `traffic-editor`is a top-down editor that allows the annotation of maps. There are two functions of the editor. The first function is to specify the "traffic lanes", which are shared amongst all fleets operating in this area. The second function is to allow the automatic generation of a gazebo world by transforming annotations into their corresponding (3D) gazebo counterparts. This is useful when we do not have a gazebo world of the are, but we have a 2D planar map.

We will focus on the first function in this tutorial, since we conviniently already have a world to work with.

## Understanding the map
It is first and formost important to understand the vendor map. For `maze.png`, we can see that the image is 400x400 pixels. In addition, we know from `maze.yaml` in `mir_gazebo/maps` that the resolution is 0.05. Finally, the map is displayed without any transformations. Thus the 2d coordinate system of this map is taken with reference to the bottom left ( as viewed in rviz ) as the origin (0, 0) and the frame extending 400 x 0.05 = 20 units in the x and y axis.

## 