// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_ROS_HPP_
#define NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_ROS_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/costmap_service_client.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_robot/robot.hpp"
#include "nav2_simple_planner/astar_planner.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"

namespace nav2_simple_planner
{

class AStarPlannerROS : public rclcpp::Node
{
public:
  AStarPlannerROS();
  ~AStarPlannerROS();

  nav2_tasks::TaskStatus computePathToPose(
    const nav2_tasks::ComputePathToPoseCommand::SharedPtr command);

private:
  AStarPlanner astar_planner_;
  std::unique_ptr<nav2_tasks::ComputePathToPoseTaskServer> task_server_;
  std::unique_ptr<nav2_robot::Robot> robot_;

  // The costmap to use
  nav2_msgs::msg::Costmap costmap_;

  // The global frame of the costmap
  std::string global_frame_;

  // Service client for getting the costmap
  nav2_tasks::CostmapServiceClient costmap_client_;

  // Request costmap from world model
  void getCostmap(
    nav2_msgs::msg::Costmap & costmap, const std::string layer = "master",
    const std::chrono::milliseconds waitTime = std::chrono::milliseconds(100));

  OccupancyGrid extractOccupancy(const nav2_msgs::msg::Costmap & costmap_);

  // Compute a plan given start and goal poses, provided in global world frame.
  bool makePlan(
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    nav2_msgs::msg::Path & plan);

  // Transform a point from world to map frame
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  // Transform a point from map to world frame
  void mapToWorld(double mx, double my, double & wx, double & wy);

  nav2_msgs::msg::Path makeROSpath(const Path & path);

    // Publish a path for visualization purposes
  void publishPlan(const nav2_msgs::msg::Path & path);
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_ROS_HPP_
