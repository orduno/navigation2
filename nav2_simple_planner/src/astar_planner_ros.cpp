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

#include <iostream>
#include <memory>

#include "nav2_simple_planner/astar_planner_ros.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"
#include "nav2_simple_planner/astar_planner.hpp"
#include "nav2_simple_planner/logger.hpp"
#include "nav2_simple_planner/point.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_simple_planner
{

AStarPlannerROS::AStarPlannerROS()
: Node("AStarPlanner"),
  global_frame_("map")
{
  RCLCPP_INFO(get_logger(), "Initializing...");

  // Initialize ROS interface
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  robot_ = std::make_unique<nav2_robot::Robot>(temp_node);

  plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>("simple_planner_plan", 1);

  task_server_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskServer>(temp_node, false),

  task_server_->setExecuteCallback(
    std::bind(&AStarPlannerROS::computePathToPose, this, std::placeholders::_1));

  // Start listening for incoming ComputePathToPose task requests
  task_server_->startWorkerThread();

  RCLCPP_INFO(get_logger(), "Completed initialization");
}

AStarPlannerROS::~AStarPlannerROS()
{
  RCLCPP_INFO(get_logger(), "Shutting down...");
}

TaskStatus
AStarPlannerROS::computePathToPose(const nav2_tasks::ComputePathToPoseCommand::SharedPtr command)
{
  nav2_tasks::ComputePathToPoseResult result;

  try {
    // Get an updated costmap
    getCostmap(costmap_);
    RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d",
      costmap_.metadata.size_x, costmap_.metadata.size_y);

    // Get the current pose from the robot
    auto start = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    // TODO(orduno) Should get the actual robot pose
    // if (!robot_->getCurrentPose(start)) {
    //   RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
    //   return TaskStatus::FAILED;
    // }

    // TODO(orduno) remove, just for testing
    start->pose.pose.position.x = 0.0;
    start->pose.pose.position.y = 0.0;
    command->pose.position.x = 9.0;
    command->pose.position.y = 9.0;

    RCLCPP_INFO(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).", start->pose.pose.position.x, start->pose.pose.position.y,
      command->pose.position.x, command->pose.position.y);

    bool foundPath = makePlan(start->pose.pose, command->pose, result);

    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Cancelled global planning task.");
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    if (!foundPath) {
      RCLCPP_WARN(get_logger(), "Planning algorithm failed to generate a valid"
        " path to (%.2f, %.2f)", command->pose.position.x, command->pose.position.y);
      return TaskStatus::FAILED;
    }

    RCLCPP_INFO(get_logger(),
      "Found valid path of size %u", result.poses.size());

    // Publish the plan for visualization purposes
    RCLCPP_INFO(get_logger(), "Publishing the valid path.");
    publishPlan(result);

    RCLCPP_INFO(get_logger(),
      "Successfully planned to (%.2f, %.2f)", command->pose.position.x, command->pose.position.y);

    task_server_->setResult(result);
    return TaskStatus::SUCCEEDED;
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Plan calculation to (%.2f, %.2f) failed: \"%s\"",
      command->pose.position.x, command->pose.position.y, ex.what());
    return TaskStatus::FAILED;
  } catch (...) {
    RCLCPP_WARN(get_logger(), "Plan calculation failed");
    return TaskStatus::FAILED;
  }
}

void
AStarPlannerROS::getCostmap(
  nav2_msgs::msg::Costmap & costmap, const std::string /*layer*/,
  const std::chrono::milliseconds /*waitTime*/)
{
  // TODO(orduno): explicitly provide specifications for costmap using the costmap on the request,
  //               including master (aggregate) layer

  // auto request = std::make_shared<nav2_tasks::CostmapServiceClient::CostmapServiceRequest>();
  // request->specs.resolution = 1.0;

  // auto result = costmap_client_.invoke(request);
  // costmap = result.get()->map;

  // TODO(orduno) remove, added just for testing
  costmap.metadata.origin.position.x = 0.0;
  costmap.metadata.origin.position.y = 0.0;
  costmap.metadata.resolution = 1.0;
  costmap.metadata.size_x = 10.0;
  costmap.metadata.size_y = 10.0;
}

bool
AStarPlannerROS::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  nav2_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  unsigned int mx, my;
  if (!worldToMap(start.position.x, start.position.y, mx, my)) {
    RCLCPP_WARN(
      get_logger(), "Cannot create a plan: the robot's start position is off the global costmap.");
    return false;
  }

  Point map_start;
  map_start.column = static_cast<int>(mx);
  map_start.row = static_cast<int>(my);

  if (!worldToMap(goal.position.x, goal.position.y, mx, my)) {
    RCLCPP_WARN(get_logger(), "The goal sent to the planner is off the global costmap.");
    return false;
  }

  Point map_goal;
  map_goal.column = static_cast<int>(mx);
  map_goal.row = static_cast<int>(my);

  // TODO(orduno)
  std::shared_ptr<OccupancyGrid> occupancy = std::make_shared<OccupancyGrid>(
    extractOccupancy(costmap_));

  if (astar_planner_.computePath(occupancy, map_goal, map_start)) {
    // logger->info() << "Occupancy: " << *occupancy;
    // logger->info() << "Path: " << planner.getPath();
    plan = makeROSpath(astar_planner_.getPath());
  }

  return !plan.poses.empty();
}

// TODO(orduno) make these part of the costmap wrapper
bool
AStarPlannerROS::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_.metadata.origin.position.x || wy < costmap_.metadata.origin.position.y) {
    RCLCPP_ERROR(get_logger(), "wordToMap failed: wx, wy: %f, %f, size_x, size_y: %d, %d", wx, wy,
      costmap_.metadata.size_x, costmap_.metadata.size_y);
    return false;
  }

  mx = static_cast<unsigned int>(
    std::round((wx - costmap_.metadata.origin.position.x) / costmap_.metadata.resolution));
  my = static_cast<unsigned int>(
    std::round((wy - costmap_.metadata.origin.position.y) / costmap_.metadata.resolution));

  if (mx < costmap_.metadata.size_x && my < costmap_.metadata.size_y) {
    return true;
  }

  RCLCPP_ERROR(get_logger(), "wordToMap failed: mx, my: %d, %d, size_x, size_y: %d, %d", mx, my,
    costmap_.metadata.size_x, costmap_.metadata.size_y);

  return false;
}

void
AStarPlannerROS::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_.metadata.origin.position.x + mx * costmap_.metadata.resolution;
  wy = costmap_.metadata.origin.position.y + my * costmap_.metadata.resolution;
}

OccupancyGrid AStarPlannerROS::extractOccupancy(const nav2_msgs::msg::Costmap & /*costmap_*/)
{
  return  OccupancyGrid {
  // 0  1  2  3  4  5  6  7  8  9
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // 0
    {0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, // 1
    {1, 0, 1, 0, 1, 0, 1, 0, 0, 1}, // 2
    {0, 0, 1, 0, 1, 0, 1, 1, 0, 0}, // 3
    {0, 0, 1, 0, 1, 0, 0, 1, 0, 0}, // 4
    {0, 0, 1, 0, 1, 1, 0, 0, 1, 0}, // 5
    {0, 0, 0, 0, 0, 0, 1, 0, 1, 0}, // 6
    {0, 0, 1, 0, 1, 0, 0, 0, 1, 0}, // 7
    {0, 0, 1, 0, 1, 1, 1, 1, 1, 0}, // 8
    {0, 0, 1, 0, 0, 0, 0, 0, 1, 0}, // 9
  };
}

nav2_msgs::msg::Path
AStarPlannerROS::makeROSpath(const Path & path)
{
  nav2_msgs::msg::Path ros_path;

  ros_path.header.stamp = this->now();
  ros_path.header.frame_id = global_frame_;

  // convert the t to world coordinates
  for (const auto & point : path.getPoints()) {
    double world_x, world_y;
    mapToWorld(point.column, point.row, world_x, world_y);

    geometry_msgs::msg::Pose pose;
    pose.position.x = world_x;
    pose.position.y = world_y;

    // Currently only 2D planning
    pose.position.z = 0.0;

    // Currently not planning robot heading
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    ros_path.poses.push_back(pose);
  }

  return ros_path;
}

void
AStarPlannerROS::publishPlan(const nav2_msgs::msg::Path & path)
{
  // Publish as a nav1 path msg
  nav_msgs::msg::Path rviz_path;

  rviz_path.header = path.header;
  rviz_path.poses.resize(path.poses.size());

  // Assuming path is already provided in world coordinates
  for (unsigned int i = 0; i < path.poses.size(); i++) {
    rviz_path.poses[i].header = path.header;
    rviz_path.poses[i].pose = path.poses[i];
  }

  plan_publisher_->publish(rviz_path);
}

}  // namespace nav2_simple_planner
