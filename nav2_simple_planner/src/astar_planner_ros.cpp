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

  world_ = std::make_shared<World>(temp_node);

  astar_planner_ = std::make_unique<AStarPlanner>(world_);

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
    world_->update();

    auto start = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    if (!getPose(start)){
      return TaskStatus::FAILED;
    }

    // TODO(orduno)
    command->pose.position.x = 9.0;
    command->pose.position.y = 9.0;

    RCLCPP_INFO(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
      "(%.2f, %.2f).", start->pose.pose.position.x, start->pose.pose.position.y,
      command->pose.position.x, command->pose.position.y);

    bool foundPath = makePath(start->pose.pose, command->pose, result);

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

    // Publish the plan for visualization purposes
    publishPlan(result);

    RCLCPP_INFO(get_logger(), "Found a valid path of size %u to (%.2f, %.2f)",
      result.poses.size(), command->pose.position.x, command->pose.position.y);

    // Send the path to the task client
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

bool
AStarPlannerROS::getPose(std::shared_ptr<geometry_msgs::msg::PoseWithCovarianceStamped> & pose){

  // TODO(orduno) Should get the actual robot pose
  // if (!robot_->getCurrentPose(start)) {
  //   RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
  //   return false;
  // }

  // TODO(orduno) remove, just for testing
  pose->pose.pose.position.x = 0.0;
  pose->pose.pose.position.y = 0.0;

  return true;
}

bool
AStarPlannerROS::makePath(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal,
  nav2_msgs::msg::Path & path)
{
  // clear the plan, just in case
  path.poses.clear();

  if (world_->isEmpty()) {
    RCLCPP_WARN(
      get_logger(), "Cannot create a path: the world model has not been initialized.");
    return false;
  }

  unsigned int mx, my;
  if (!world_->worldToMap(start.position.x, start.position.y, mx, my)) {
    RCLCPP_WARN(
      get_logger(), "Cannot create a path: the robot's start position is off the global costmap.");
    return false;
  }

  Point map_start;
  map_start.column = static_cast<int>(mx);
  map_start.row = static_cast<int>(my);

  if (!world_->worldToMap(goal.position.x, goal.position.y, mx, my)) {
    RCLCPP_WARN(get_logger(), "The goal sent to the planner is off the global costmap.");
    return false;
  }

  Point map_goal;
  map_goal.column = static_cast<int>(mx);
  map_goal.row = static_cast<int>(my);

  if (astar_planner_->computePath(map_goal, map_start)) {
    path = makeROSPath(astar_planner_->getPath());
  }

  return !path.poses.empty();
}

nav2_msgs::msg::Path
AStarPlannerROS::makeROSPath(const Path & path)
{
  nav2_msgs::msg::Path ros_path;

  ros_path.header.stamp = this->now();
  ros_path.header.frame_id = global_frame_;

  if (world_->isEmpty()) {
    RCLCPP_WARN(
      get_logger(), "Cannot create a path: the world model has not been initialized.");
    return ros_path;
  }

  // convert the t to world coordinates
  for (const auto & point : path.getPoints()) {
    double world_x, world_y;
    world_->mapToWorld(point.column, point.row, world_x, world_y);

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
