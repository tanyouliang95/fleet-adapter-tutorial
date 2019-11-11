# rmf_core Overview
`git clone --recursive git@github.com:cnboonhan94/fleet-adapter-tutorial.git -b 03-rmf-core-overview`

Until now, we have focused on extending the vendor fleet. Our ultimate aim is to generate **feasible trajectory solutions considering the operations of all other RMF fleets.** Our current setup can only generate feasible trajectories given the vendor fleet is the only fleet in operation. We take the following steps to achieve our goals:
* Relate the vendor fleet map to a RMF Graph, a high level abstraction of the traffic lanes 
* Relate the vendor fleet movements to an RMF Trajectory, a high level abstraction of fleet motion.
* Factor information about other fleets in the environment in an RMF Schedule, in order to generate feasible trajectories.

We focus on the third point here. specifically, how RMF stores information about other fleets in the environment. 

## Trajectory
A [Trajectory](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/Trajectory.hpp) is an RMF abstraction of the space-time occupation of a robot. Usage is documented in the [tests](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/test/unit/test_Trajectory.cpp). A trajectory is a sequence of [Segments]([Segments](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/Trajectory.hpp#L189)). Each Segment encapsulates the following information:
* `finish_position`
* `finish_velocity`
* `finish_time`
* `Profile`
A sequence of Segments represents the space-time occupation of a robot. the finish_ properties represent the time, while the Profile represents space. A [Profile](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/Trajectory.hpp#L63) has the following properties:
* `shape`: Represents space occupied
* `Agency`: Represents the expected behaviour of the robot in following the Trajectory in reality.

We aim to represent fleet motion along n edges in an RMF graph as a single Trajectory consisting of n+1 Segments. In particular, if we set each segment `finish_position` and `finish_velocity` properties appropriately, we can represent straight line motion between vertices. This might help simple the task of creating these trajectories.

## Query
A [Query](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Query.hpp) class encapsulates a query to the RMF Schedule. Usage is documented in the [tests](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/test/unit/schedule/test_Query.cpp). The Schedule then returns a set of `Changes` (called a `Patch`) that meet the query criteria. There are the following filters:
* `Spacetime`: Specifies the [spacetime](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Query.hpp#L46) criteria of the results. Can be All / Regions / Timespan
  * `All`: No filters in this aspect.
  * `Regions`: Only return results that intersect in the specified `Regions`.
  *  `TimeSpan`: Only return results are are active in the specified timespan.
* `Versions`: Specifies the [version](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Query.hpp#L273) of the Schedule that is relevant. The schedule updates over time. This filters to return only `Changes` that are introduced after a specific Scheduler version.
  * `All`: No filters in this aspect
  * `After`: Get every version after the specified version.

We will focus on the make_ [API](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Query.hpp#L388) which is used to generate Queries with specific criteria. Also, note that fleet adapters will be expected to `query_everything` when querying for the first time, for initialization.

## Viewer
A viewer is a class that allows users to see trajectories that are part of the RMF Schedule. It allows users to look at / iterate through trajectories. This class cannot be instantiated directly, but is the subclass of `Database` and `Mirror`.

## Database
The core of the RMF scheduler is a [Database](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Database.hpp). Usage is documented in the [tests](https://github.com/osrf/rmf_core/blob/63b8334de3819ccf09583f5ce01f4d73d3042f33/rmf_traffic/test/unit/schedule/test_Database.cpp). Here, a set of trajectories currently being executed by RMF fleets are stored. It inherits the `Viewer` class. 
* We can use [`[insert  | interrupt | delay | replace | erase | cull]`](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Database.hpp#L368) functions to modify this database. They then return an incremented database version.
* The [`Change`](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Database.hpp#L44) class flags a change that occurs in the database. There are a few possibilities, representing how the database trajectories change.
* A [`Patch`](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Database.hpp#L327) is a container that bundles multiple database `Changes`. We can [query the database](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Database.hpp#L363) to see what changes were made given some criteria.\

## Mirror
A [Mirror](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Mirror.hpp) is a local copy of the scheduled trajectories to processes that do not contain the original upstream Database copy, and provides a relevant subset of the Database. Usage is documented in the [tests](https://github.com/osrf/rmf_core/blob/63b8334de3819ccf09583f5ce01f4d73d3042f33/rmf_traffic/test/unit/schedule/test_Mirror.cpp). The [`update`](https://github.com/osrf/rmf_core/blob/traffic_msgs/rmf_traffic/include/rmf_traffic/schedule/Mirror.hpp#L43) function thus performs this update. 

Additionally, before submitting a trajectory to the scheduler, a fleet adapter should check for conflicts using the `Viewer` in the `Mirror`.