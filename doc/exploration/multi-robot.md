# Navigation on Multi-robot Systems

## Objective
Engage on an exploratory discussion of opportunities and challenges within Multi-Robot Systems (MRS) under ROS2.

## Current status

The `nav2` stack supports multiple robots co-existing on a simulated environment sharing a ROS domain.

The `nav2_multi_tb3_simulation_launch.py` launch file in the `nav2_bringup` package is provided as an example for launching a multi-robot system in simulation. The following steps are executed during launch-time:
- Spawn two robots into a Gazebo instance.
- Launch a namespaced `nav2` instance per robot.
- Launch an RVIZ instance per robot with plugins to start, stop, reset, etc the stacks independently, and provide initial and goal poses.

## Key Questions

- Consider the system below


- Beyond supporting multiple `nav2` instances on a shared ROS domain, what other aspects should be considered...

- Are changes needed in `nav2` to support/adapt to MRS?

- What are the use cases to consider?

- What is within scope? What is out of scope?

- What are some of the most important features and capabilities?

- What approach should we take?

- How to partition the system?

    ROS2 is missing some [features](https://index.ros.org//doc/ros2/Roadmap/#new-features) to partition multi-robot systems.

    Currently, instances of the nav stack are partition using namespaces. However,
    nodes across robots share the same domain and therefore have to discover each other (DDS discovery) and data is accessible across it's undesired that nodes across robots share the same domain. Other logical partitioning methods have been suggested:
    - DDS Keyed Data
    - DDS Domain and Participants

    Real hardware systems might have separate ROS domains but will have to define a channel through which data is shared.

  - Besides logical partitioning, there might be other physical solutions.

- Any other considerations?

## Known risks/issues
- Limitations with the number of robots we're able simulate successfully. Bringup of individual systems becomes slow as it grows (discovery issues with DDS?)

## Existing Solutions

## Next Steps
- Update tutorial with multi-robot example.
- Create a multi-robot system test.
- Design Improvements
  - Currently transforms are on separate topics (`/robot1/tf`, `/robot2/tf`, etc.), create a node that will take TF trees from robots , create a single tf tree (`/system/tf`) with transforms from `map` to each robot's `base_link` and broadcast it
- Usability Improvements
  - Multi-robot on a single RVIZ instance using a plugin to toggle between robots (dropdown menu for example) for setting initial pose and goal pose. Visualization of robots positions using the system-level transform (`/system/tf`).
- Performance Improvements
  - Explore limitations for launching more robots.
  - Create a `RobotsLayer` in `Costmap2D` which takes the system-level transform (and footprint) and populates regions occupied by robots.
- Launching improvements
  - Remove topic remapping from lower-level files by using a [`PushNodeRemapping`](https://github.com/ros2/launch_ros/issues/56) action only at the multi-robot launch file.
  - Move topics with hard-coded absolute namespace (`/map`, `/cmd_vel`, etc) to relative ones and remove remappings for those.
  - Have `nav2_common::Node` derive from `launch_ros::Actions::Node` and avoid code duplication.
