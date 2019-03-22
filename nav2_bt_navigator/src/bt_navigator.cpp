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

#include "nav2_bt_navigator/bt_navigator.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

#include "nav2_tasks/bt_conversions.hpp"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: nav2_lifecycle::LifecycleNode("bt_navigator")
{
  RCLCPP_INFO(get_logger(), "Creating");
}

BtNavigator::~BtNavigator()
{
  RCLCPP_INFO(get_logger(), "Destroying");
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Create the NavigateToPose task server for this node and set the callback
  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(shared_from_this());
  task_server_->on_configure(state);
  task_server_->setExecuteCallback(
    std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<NavigateToPoseBehaviorTree>(shared_from_this());

  // Create the path that will be returned from ComputePath and sent to FollowPath
  goal_ = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  path_ = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create<BT::BlackboardLocal>();

  // Put items on the blackboard
  blackboard_->set<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("goal", goal_);  // NOLINT
  blackboard_->set<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path", path_);  // NOLINT
  blackboard_->set<nav2_lifecycle::LifecycleNode::SharedPtr>("node", shared_from_this());  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("node_loop_timeout", std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);  // NOLINT

  // Get the BT filename to use from the node parameter
  std::string bt_xml_filename;
  get_parameter_or<std::string>(std::string("bt_xml_filename"), bt_xml_filename,
    std::string("bt_navigator.xml"));

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return nav2_lifecycle::CallbackReturn::FAILURE;
  }

  xml_string_ = std::string(std::istreambuf_iterator<char>(xml_file),
      std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(get_logger(), "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(get_logger(), "Behavior Tree XML: %s", xml_string_.c_str());

  // Create the Behavior Tree from the XML input (after registering our own node types)
  tree_ = bt_->buildTreeFromText(xml_string_, blackboard_);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  task_server_->on_activate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  task_server_->on_deactivate(state);

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  task_server_->on_cleanup(state);
  task_server_.reset();

  path_.reset();
  blackboard_.reset();
  xml_string_.clear();

  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Handling error state");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

nav2_lifecycle::CallbackReturn
BtNavigator::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_lifecycle::CallbackReturn::SUCCESS;
}

TaskStatus
BtNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Begin navigating from current location to (%.2f, %.2f)",
    command->pose.position.x, command->pose.position.y);

  // Get the goal pointer off of the blackboard...
  auto goal = blackboard_->get<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("goal");

  // and update it with the incoming command
  *goal = *command;

  // Execute the BT that was previously created in the configure step
  TaskStatus result = bt_->run(tree_,
      std::bind(&nav2_tasks::NavigateToPoseTaskServer::cancelRequested, task_server_.get()));

  if (result == TaskStatus::CANCELED) {
    // Even though the BT is no longer running, remote actions may still be executing. So,
    // an explicit canceling of all actions allows the task clients to send cancel messages
    // to their corresponding task servers
    bt_->cancelAllActions(tree_.root_node);

    // Reset the BT so that it can be run again in the future
    bt_->resetTree(tree_.root_node);
  }

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
