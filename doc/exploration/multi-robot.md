# Multi-Robot Systems under ROS2

## Objective

Engage on an exploratory discussion to identify challenges and benefits from developing a Multi-Robot Systems (*MRS*) under ROS2.

In general we need to understand:

- What is required from ROS2 to develop an MRS? What is already there and what's missing?
- What is required from the navigation sub-system and from `ros2/nav2`?
- What is required from a ROS2-based MRS package?

## Navigation for Multi-robot Systems

In general, a navigation sub-system provides:
  - Execution of elemental navigation tasks: point-to-point, keep pose, etc.
  - Execution of composed navigation tasks: way-point following, area coverage, etc.
  - Execution of a sequence (possibly chained) navigation tasks

Where `ros2/nav2` provides *some* of the core functionality.

### Current status

The `nav2` stack currently supports multiple robots co-existing on a simulated environment sharing a ROS domain and provides point-to-point navigation for each robot.

The `nav2_bringup` package provides the `nav2_multi_tb3_simulation_launch.py` file as an example for launching a multi-robot system in simulation. The following steps are executed during launch-time:
- Spawn two robots into a Gazebo instance.
- Launch a namespaced `nav2` instance per robot.
- Launch an RVIZ instance per robot with plugins/controls to change the states of the `nav2` instances independently, and to provide the initial and goal poses.

### Requirements

As a starting point, let's clarify the following questions:

*These are meant to trigger discussion and will certainly change as we gather input from the community.*

- What is **within scope**? i.e. considering a system of multiple mobile robots, what are the concerns that *navigation* should provide a solution for?
  - Collision and deadlock avoidance between robots
  - Traffic control
  - On a centralized architecture, solutions for:
    - Centralized global path planning
    - Global world modeling and aggregation of sensing data
  - On a distributed architecture, solutions for:
    - Allocating navigation tasks between a team of robots

- What is **out of scope**?
  - Non-navigation task execution, i.e. manipulation, etc.
  - Non-navigation task allocation and mission planning

- What other features or capabilities should be considered to ensure successful navigation on an environment with multiple robots?

### Use Cases
- What are some **key use cases** to consider?
  - Robots delivering product on a warehouse or retail space.
  - Robots cleaning the floor of large building.
  - Robots collecting trash from bins.
  - Robots searching for objects.
  - Robots performing surveillance.
  - Robots creating a map of an area.
  - Two robots moving an object.

### Known risks/issues/limitations

### Next Steps

<!--
## Approach
- Improve some of the existing capabilities
- Engage on an exploratory discussion. Come to an agreement on answers to key questions.
- Refine use cases.
- Define requirements, identify building blocks, define design
- Define milestones
- ... -->

- Engage the ROS community on an **exploratory** discussion around `nav2` support for MRS.
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


## ROS2 support for MRS:

### Requirements

- What components or features are needed from ROS to develop an MRS?
  - Bridging between ROS domains, i.e. communication mechanism for robots in separate domains
    - Separating domains might be better for performance: hiding messages not required by other robots, etc.
  - Partitioning within a shared ROS domain.
  - TF2 namespacing/prefixing

- What is available?
  - Logical partitioning using namespaces on a shared domain.

- What is currently missing?
  - Complete logical [partitioning](https://index.ros.org//doc/ros2/Roadmap/#new-features). Currently, nodes sharing a domain must discover each other (DDS discovery) which presents limitations. Also data is accessible across nodes of different robots which can be undesirable. Use of DDS features have been suggested to extend the partitioning:
    - DDS Keyed Data
    - DDS Domain and Participants

    Besides logical partitioning, there might be other physical solutions.

### Known risks/issues
Simulation
- Limitations with the number of `nav2` instances we're able to bring-up successfully. -- bring-up becomes slow or eventually halts (discovery issues with DDS?)

ROS2
- Partitioning (see above)

## Support from an MRS package:

### Requirements

- What capabilities are needed?

  - A framework/stack for describing and developing a full multi-robot system:
    - Collective behavior (cooperative, competitive)
    - Robot awareness -- information that each robot has about it's team mates
    - Coordination protocol -- explicit interaction rules.
    - Organization level (centralized, distributed, hybrid)
    - Mission Planning and Execution.
    - Task decomposition and allocation model.
    - Fleet management

- What packages are available for MRS support? What capabilities are offered?
  - [Multi-robot collision avoidance](http://wiki.ros.org/multi_robot_collision_avoidance) (ROS1), more info [here](http://www.willowgarage.com/blog/2012/07/23/multi-robot-collision-avoidance)
  - [RL based multi-robot collision avoidance](https://github.com/vincekurtz/CollisionAvoidance)
  - [micROS](http://wiki.ros.org/micros_mars_task_alloc) (ROS1). Pkg for multi-task allocation based on the ALLIANCE model, more info [here](https://micros.trustie.net/) and repo [here](https://github.com/liminglong/micros_mars_task_alloc)
  - [TUW_multi_robot](https://github.com/tuw-robotics/tuw_multi_robot) (ROS1). Pkg for planning routes of multiple robots on a search graph.
  - [explore_multirobot](http://wiki.ros.org/explore_multirobot) (ROS1) for frontier exploration.
  - [map_merging](http://wiki.ros.org/map_merging) (ROS1)
  - [tf_splitter](http://wiki.ros.org/tf_splitter) (ROS1)

  Just to mention some, [here](https://github.com/topics/multi-robot) are more.

  ### Known risks/issues