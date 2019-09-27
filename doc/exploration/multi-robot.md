# Navigation for Multi-robot Systems
---
## Objective

Understand the **level of support** that `ros2/nav2` should/can provide for developing Multi-Robot Systems (*MRS*).

Additionally, engage on an exploratory discussion to identify challenges and opportunities for developing an MRS under ROS2.

---
## Current status

The `nav2` stack currently supports multiple robots co-existing on a simulated environment sharing a ROS domain.

The `nav2_multi_tb3_simulation_launch.py` launch file in the `nav2_bringup` package is provided as an example for launching a multi-robot system in simulation. The following steps are executed during launch-time:
- Spawn two robots into a Gazebo instance.
- Launch a namespaced `nav2` instance per robot.
- Launch an RVIZ instance per robot with plugins to start, stop, reset, etc the stacks independently, and provide initial and goal poses.

---
## Key Questions

<u>Navigation support for MRS:</u>

- What is **within scope** for navigation considering an environment with multiple robots?
  - *Reliable and safe execution of navigation tasks.*
  - *Collision and deadlock avoidance between robots -> Traffic control.*
  - *Execution of mission with navigation tasks involving multiple robots.*
<br>

- What is **out of scope**?
  - *Execution of other types of tasks, i.e. manipulation, perception.*
  - *Execution of a mission with multiple types fo tasks involving one or more robots.*
<br>

- What elements within scope are **currently not handled**?
  - *Mission execution, Traffic control*
<br>

- What other elements, features, capabilities should be considered to ensure successful navigation on an environment with multiple robots?
<br>

- What are some key **use cases** to consider?
  - *Robots delivering product on a warehouse or retail space.*
  - *Robots cleaning the floor of large building.*
  - *Robots collecting trash from bins.*
  - *Robots searching for objects.*
  - *Robots performing surveillance.*
  - *Robots creating a map of an area.*
  - *Two robots moving an object.*
<br>

<u>ROS2 support for MRS:</u>

- What components or features are needed from the middleware to develop an MRS?
  - *Communication mechanism for robots in separate domains.*
  - *Partitioning within a shared ROS domain:*
<br>

- What is available?
  - *Logical partitioning using namespaces on a shared domain.*
<br>

- What is currently missing?
  - *Complete [partitioning](https://index.ros.org//doc/ros2/Roadmap/#new-features). Currently, nodes sharing a domain must discover each other (DDS discovery) which presents limitations. Also data is accessible across nodes of different robots which can be undesirable. Use of DDS features have been suggested to extend teh partitioning:*
    - *DDS Keyed Data*
    - *DDS Domain and Participants*
    *Besides logical partitioning, there might be other physical solutions.
<br>

- What packages are available for MRS coordination? What capabilities are offered?
<br>

- What capabilities are needed?
  - *A model for describing and developing a full multi-robot system:*
    - *Collective behavior (cooperative, competitive)*
    - *Robot awareness -- information that each robot has about it's team mates*
    - *Coordination protocol -- explicit interaction rules.*
    - *Organization level (centralized, distributed, hybrid)*
    - *Mission Planning and Execution. Task decomposition and allocation model.*
    - *Fleet management*
<br>

---
## Known risks/issues
- Limitations with the number of robots we're able simulate successfully. Bringup of individual systems becomes slow or eventually halts as it grows (discovery issues with DDS?)

---
## Approach
- Improve some of the existing capabilities, see next section.


---
## Next Steps
- Update tutorial to include the multi-robot example.
- Add multi-robot system test to `nav2`.
- Design Improvements
  - Currently transforms are on separate topics (`/robot1/tf`, `/robot2/tf`, etc.), create a node that will take TF trees from robots , create a single tf tree (`/system/tf`) with transforms from `map` to each robot's `base_link` and broadcast it
- Usability Improvements
  - Multi-robot on a single RVIZ instance using a plugin to toggle between robots (dropdown menu for example) for setting initial pose and goal pose. Visualization of robots positions using the system-level transform (`/system/tf`).
- Performance Improvements
  - Explore limitations for launching more robots. Test with different DDS implementations.
  - Create a `RobotsLayer` in `Costmap2D` which takes the system-level transform (and footprint) and populates regions occupied by robots.
- Launching improvements
  - Remove topic remapping from lower-level files by using a [`PushNodeRemapping`](https://github.com/ros2/launch_ros/issues/56) action only at the multi-robot launch file.
  - Move topics with hard-coded absolute namespace (`/map`, `/cmd_vel`, etc) to relative ones and remove remappings for those.
  - Have `nav2_common::Node` derive from `launch_ros::Actions::Node` and avoid code duplication.
