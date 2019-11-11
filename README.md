# RMF_Core Overview
Until now, we have focused on extending the vendor fleet. Our ultimate aim is to generate **feasible trajectory solutions considering the operations of all other RMF fleets.** Our current setup can only generate feasible trajectories given the vendor fleet is the only fleet in operation. We take the following steps to achieve our goals:
* Relate the vendor fleet map to a RMF Graph, a high level abstraction of the traffic lanes ( Done in 03-Parse-Yaml-to-Graph )
* Relate the vendor fleet movements to an RMF Trajectory, a high level abstraction of fleet motion.
* Consider information about other fleets in the environment in an RMF Schedule, in order to generate feasible trajectories.

We focus on the second and third point here, in particular, how to encapsulate information about other fleets in the environment. 

## Trajectory
A trajectory is an RMF abstraction of the space-time occupation of a robot. Usage is documented in the tests, under `ros2/src/rmf_core/rmf_traffic/test/unit/test_Trajectory.cpp`. A trajectory is a sequence of Segments. Each Segment encapsulates the following information:
* `finish_position`
* `finish_velocity`
* `finish_time`
* `Profile`
A sequence of Segments represents the space-time occupation of a robot. the finish_ properties represent the time, while the Profile represents space. A Profile has the folloing properties:
* `shape`: Represents space occupied
* `Agency`: Represents the expected behaviour of the robot in following the Trajectory in reality.

We aim to represent fleet motion between n edges in an RMF graph as a single Trajectory consisting of n Segments. In particular, if we set each segment `finish_position` and `finish_velocity` properties appropriately, we can represent straight line motion between vertices. This might help simple the task of creating these trajectories.

## Database
The core of the schedule is a database. Usage is documented in the tests, under `ros2/src/rmf_core/rmf_traffic/test/unit/schedule/test_Database.cpp`. Here, a set of trajectories currently being executed by RMF fleets are stored.