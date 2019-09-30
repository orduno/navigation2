# Navigation for Multi-robot Systems
---
## Objective

Understand the **level of support** that `ros2/nav2` should/can provide for developing Multi-Robot Systems (*MRS*).

Additionally, engage on an exploratory discussion to identify challenges and opportunities for developing an MRS under ROS2.

---
## Current status

The `nav2` stack currently supports multiple robots co-existing on a simulated environment sharing a ROS domain.

The `nav2_bringup` package provides the `nav2_multi_tb3_simulation_launch.py` file as an example for launching a multi-robot system in simulation. The following steps are executed during launch-time:
- Spawn two robots into a Gazebo instance.
- Launch a namespaced `nav2` instance per robot.
- Launch an RVIZ instance per robot with plugins to change the states of the `nav2` instances independently, and to provide the initial and goal poses.

---
## Key Questions

*Take the current answers as a starting point, these are meant to trigger discussion and will certainly change as we gather input from the community.*

<u>Navigation support for MRS:</u>

- What is **within scope**? i.e. considering a system of multiple mobile robots, what are the concerns that *navigation* should provide a solution for?
<span style="color:rgba(51,73, 255);">
  - Execution of elemental navigation tasks: point-to-point, keep pose, etc.
  - Execution of composed navigation tasks: way-point following, area coverage, etc.
  - Execution of a sequence of navigation tasks
  - Collision and deadlock avoidance between robots
  - Traffic control
  - On a centralized architecture, solutions for:
    - Centralized planning
    - Global world modeling and aggregation of sensing data
  - On a distributed architecture, solutions for:
    - Allocating navigation tasks between a team of robots
</span>
<!-- <br> -->

- What is **out of scope**?
<span style="color:rgba(51,73, 255);">
  - Execution of tasks outside of navigation, i.e. manipulation, perception.
  - Execution of a mission plan (sequence of tasks) with a mix of tasks on a single robot.
  - Allocation of non-navigation tasks on a team of robots.
  - Fleet management
</span>
<!-- <br> -->

- What other elements, features, capabilities should be considered to ensure successful navigation on an environment with multiple robots?
<span style="color:rgba(51,73, 255);"> </span>
<!-- <br> -->

- What are some key **use cases** to consider?
<span style="color:rgba(51,73, 255);">
  - Robots delivering product on a warehouse or retail space.
  - Robots cleaning the floor of large building.
  - Robots collecting trash from bins.
  - Robots searching for objects.
  - Robots performing surveillance.
  - Robots creating a map of an area.
  - Two robots moving an object.
</span>
<!-- <br> -->

<u>ROS2 support for MRS:</u>

- What components or features are needed from the middleware to develop an MRS?
<span style="color:rgba(51,73, 255);">
  - Communication mechanism for robots in separate domains.
  - Partitioning within a shared ROS domain.
</span>
<!-- <br> -->

- What is available?
<span style="color:rgba(51,73, 255);">
  - Logical partitioning using namespaces on a shared domain.
</span>
<!-- <br> -->

- What is currently missing?
<span style="color:rgba(51,73, 255);">
  - Complete [partitioning](https://index.ros.org//doc/ros2/Roadmap/#new-features). Currently, nodes sharing a domain must discover each other (DDS discovery) which presents limitations. Also data is accessible across nodes of different robots which can be undesirable. Use of DDS features have been suggested to extend the partitioning:
    - DDS Keyed Data
    - DDS Domain and Participants

    Besides logical partitioning, there might be other physical solutions.
</span>
<!-- <br> -->

<u>Support from an MRS package:</u>

- What capabilities are needed?
<span style="color:rgba(51,73, 255);">
  - A model for describing and developing a full multi-robot system:
    - Collective behavior (cooperative, competitive)
    - Robot awareness -- information that each robot has about it's team mates
    - Coordination protocol -- explicit interaction rules.
    - Organization level (centralized, distributed, hybrid)
    - Mission Planning and Execution. Task decomposition and allocation model.
    - Fleet management
</span>
<!-- <br> -->

- What packages are available for MRS coordination? What capabilities are offered?
<span style="color:rgba(51,73, 255);"> </span>
<!-- <br> -->

---
## Known risks/issues
- Limitations with the number of robots we're able simulate successfully; bringup of individual `nav2 ` instances becomes slow or eventually halts (discovery issues with DDS?)

<!-- ---
## Approach
- Improve some of the existing capabilities, see next section.
- Engage on an exploratory discussion. Come to an agreement on answers to key questions.
- Refine use cases.
- Define requirements, identify building blocks, define design
- Define milestones
- ... -->

---
## Next Steps
- Engage the ROS community on an **exploratory** discussion around MRS and `nav2` support.
- Update **tutorial** to include the multi-robot example.
- Add **multi-robot system test** to `nav2`.
- Design Improvements
  - **Unified transform tree**. Currently transforms are on separate topics (`/robot1/tf`, `/robot2/tf`, etc.), create a node that will take TF trees from robots , create a single tf tree (`/system/tf`) with transforms from `map` to each robot's `base_link`.
  - **Single map server**
- Usability Improvements
  - Multi-robot on a **single RVIZ** instance using a plugin to toggle between robots (dropdown menu for example) for setting initial pose and goal pose. Visualization of robots positions using the system-level transform (`/system/tf`).
- Performance Improvements
  - Explore **limitations with launching more robots**. Test with different DDS implementations.
  - **Minimal collision avoidance**. Create a `RobotsLayer` in `Costmap2D` which takes the system-level transform (and footprint) and populates regions occupied by robots.

In addition there are a few *minor* improvements to be done on the **launch files**:
  - Remove topic remapping from lower-level files by using a [`PushNodeRemapping`](https://github.com/ros2/launch_ros/issues/56) action only at the multi-robot launch file.
  - Move topics with hard-coded absolute namespace (`/map`, `/cmd_vel`, etc) to relative ones and remove remappings for those.
  - Have `nav2_common::Node` derive from `launch_ros::Actions::Node` and avoid code duplication.
