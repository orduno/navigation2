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

#include "nav2_simple_navigator/simple_navigator.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>

using namespace std::chrono_literals;

namespace nav2_simple_navigator
{

SimpleNavigator::SimpleNavigator()
: nav2_lifecycle::LifecycleNode("simple_navigator", "", true)
{
  RCLCPP_INFO(get_logger(), "Creating");
}

SimpleNavigator::~SimpleNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("simple_navigator_client_node");

  self_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
    client_node_, "navigate_to_pose");

  goal_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::PoseStamped>("goal",
      std::bind(&SimpleNavigator::onGoalPoseReceived, this, std::placeholders::_1));

  // Create our two task clients
  planner_client_ =
    std::make_unique<nav2_util::SimpleActionClient<nav2_msgs::action::ComputePathToPose>>(
    client_node_, "compute_path_to_pose");

  controller_client_ =
    std::make_unique<nav2_util::SimpleActionClient<nav2_msgs::action::FollowPath>>(
    client_node_, "follow_path");

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "navigate_to_pose",
      std::bind(&SimpleNavigator::navigateToPose, this, std::placeholders::_1));

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  planner_client_.reset();
  controller_client_.reset();
  action_server_.reset();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(get_logger(), "Handling error state");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

void
SimpleNavigator::navigateToPose(const std::shared_ptr<GoalHandle> goal_handle)
{
  // Initialize the NavigateToPose goal and result
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Set up the input and output for the global planner
  nav2_msgs::action::ComputePathToPose::Goal planner_goal;
  planner_goal.pose = goal->pose;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::ComputePathToPose>::WrappedResult
    planner_result;

  RCLCPP_DEBUG(get_logger(), "Getting the path from the planner for goal pose:");
  RCLCPP_DEBUG(get_logger(), "position.x: %f", goal->pose.pose.position.x);
  RCLCPP_DEBUG(get_logger(), "position.y: %f", goal->pose.pose.position.y);
  RCLCPP_DEBUG(get_logger(), "position.z: %f", goal->pose.pose.position.z);
  RCLCPP_DEBUG(get_logger(), "orientation.x: %f", goal->pose.pose.orientation.x);
  RCLCPP_DEBUG(get_logger(), "orientation.y: %f", goal->pose.pose.orientation.y);
  RCLCPP_DEBUG(get_logger(), "orientation.z: %f", goal->pose.pose.orientation.z);
  RCLCPP_DEBUG(get_logger(), "orientation.w: %f", goal->pose.pose.orientation.w);

  // Get the path from the global planner
  auto status = planner_client_->invoke(planner_goal, planner_result);
  if (status != nav2_util::ActionStatus::SUCCEEDED) {
    goal_handle->set_aborted(result);
    return;
  }

  RCLCPP_INFO(get_logger(), "Received path of size %u from planner",
    planner_result.result->path.poses.size());

  // Print the path received from the planner
  int index = 0;
  for (auto pose : planner_result.result->path.poses) {
    RCLCPP_DEBUG(get_logger(), "Point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  // Set up the input and output for the local planner
  nav2_msgs::action::FollowPath::Goal controller_goal;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowPath>::WrappedResult controller_result;
  controller_goal.path = planner_result.result->path;

  RCLCPP_INFO(get_logger(), "Sending path to the controller to execute");

  // Call the local planner to follow the path
  controller_client_->send_goal(controller_goal);
  for (;; ) {
    auto rc = controller_client_->wait_for_result(100ms);
    switch (rc) {
      case nav2_util::ActionStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Control task completed, navigation successful");
        goal_handle->set_succeeded(result);
        return;

      case nav2_util::ActionStatus::FAILED:
        RCLCPP_INFO(get_logger(), "Control task failed, returning");
        goal_handle->set_aborted(result);
        return;

      case nav2_util::ActionStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Control task has been canceled, returning");
        goal_handle->set_canceled(result);
        return;

      case nav2_util::ActionStatus::RUNNING:
        // While the local planner is executing, check to see if this action (navigation) has
        // been canceled. If so, cancel the child task and then update this actions's status
        if (goal_handle->is_canceling()) {
          RCLCPP_INFO(get_logger(), "Navigation task has been canceled");
          controller_client_->cancel();
          goal_handle->set_canceled(result);
          return;
        }

        // TODO(mjeronimo): handle pre-emption
        break;

      default:
        throw std::logic_error("Invalid status value");
    }
  }
}

void
SimpleNavigator::onGoalPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose = *pose;
  self_client_->async_send_goal(goal);
}

}  // namespace nav2_simple_navigator
