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
using nav2_tasks::TaskStatus;

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
SimpleNavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Create our two task clients
  auto node = shared_from_this();
  planner_client_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskClient>(node);
  controller_client_ = std::make_unique<nav2_tasks::FollowPathTaskClient>(node);

  // Configure them
  planner_client_->on_configure(state);
  controller_client_->on_configure(state);

  // Create the action server that we implement with our navigateToPose method
  action_server_ = std::make_unique<ActionServer>(rclcpp_node_, "navigate_to_pose",
      std::bind(&SimpleNavigator::navigateToPose, this, std::placeholders::_1));

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  planner_client_->on_activate(state);
  controller_client_->on_activate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  planner_client_->on_deactivate(state);
  controller_client_->on_deactivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
SimpleNavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  planner_client_->on_cleanup(state);
  controller_client_->on_cleanup(state);

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
  RCLCPP_INFO(get_logger(), "navigateToPose");

  // Initialize the NavigateToPose goal and result
  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();

  // TODO(mjeronimo): handle or reject an attempted pre-emption

  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    goal->pose.pose.position.x, goal->pose.pose.position.y);

  // Create the path to be returned from ComputePath and sent to the FollowPath task
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Create the input for the global planner
  auto planning_goal = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  *planning_goal = goal->pose;

  RCLCPP_INFO(get_logger(), "Getting the path from the planner for goal pose:");
  RCLCPP_INFO(get_logger(), "position.x: %f", goal->pose.pose.position.x);
  RCLCPP_INFO(get_logger(), "position.y: %f", goal->pose.pose.position.y);
  RCLCPP_INFO(get_logger(), "position.z: %f", goal->pose.pose.position.z);
  RCLCPP_INFO(get_logger(), "orientation.x: %f", goal->pose.pose.orientation.x);
  RCLCPP_INFO(get_logger(), "orientation.y: %f", goal->pose.pose.orientation.y);
  RCLCPP_INFO(get_logger(), "orientation.z: %f", goal->pose.pose.orientation.z);
  RCLCPP_INFO(get_logger(), "orientation.w: %f", goal->pose.pose.orientation.w);

  planner_client_->sendCommand(planning_goal);

  // Loop until the subtasks are completed
  for (;; ) {
    // Check to see if this task (navigation) has been canceled. If so, cancel any child
    // tasks and then cancel this task

    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Canceling navigation task");
      planner_client_->cancel();
      goal_handle->set_canceled(result);
      return;
    }

    // Check if the planning task has completed
    TaskStatus status = planner_client_->waitForResult(path, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Calculated global plan to reach (%.2f, %.2f)",
          goal->pose.pose.position.x, goal->pose.pose.position.y);
        goto planning_succeeded;

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Planning task failed");
        goal_handle->set_aborted(result);
        return;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Planning task still running");
        break;

      case TaskStatus::CANCELED:  // Can't be canceled by anyone else (we own the goal handle)
      default:
        throw std::logic_error("Invalid status value");
    }
  }

planning_succeeded:

  RCLCPP_INFO(get_logger(), "Received path of size %u from planner", path->poses.size());

  int index = 0;
  for (auto pose : path->poses) {
    RCLCPP_DEBUG(get_logger(), "Point %u x: %0.2f, y: %0.2f",
      index, pose.position.x, pose.position.y);
    index++;
  }

  RCLCPP_INFO(get_logger(), "Sending path to the controller to execute");

  controller_client_->sendCommand(path);

  // Loop until the control task completes
  for (;; ) {
    // Check to see if this action (navigation) has been canceled. If so, cancel any child
    // tasks and then update this actions's status
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(get_logger(), "Navigation task has been canceled");
      controller_client_->cancel();
      goal_handle->set_canceled(result);
      return;
    }

    // Check if the control task has completed
    auto controlResult = std::make_shared<nav2_tasks::FollowPathResult>();
    TaskStatus status = controller_client_->waitForResult(controlResult, 100ms);

    switch (status) {
      case TaskStatus::SUCCEEDED:
        {
          RCLCPP_INFO(get_logger(), "Control task completed");

          // This is an empty message, so there are no fields to set
          goal_handle->set_succeeded(result);
          return;
        }

      case TaskStatus::FAILED:
        RCLCPP_ERROR(get_logger(), "Control task failed");
        goal_handle->set_aborted(result);
        return;

      case TaskStatus::RUNNING:
        RCLCPP_DEBUG(get_logger(), "Control task still running");
        break;

      case TaskStatus::CANCELED:
      default:
        throw std::logic_error("Invalid status value");
    }
  }
}

}  // namespace nav2_simple_navigator
