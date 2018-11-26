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

#include <chrono>
#include <thread>

#include "nav2_dummy_planner/dummy_planner.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_dummy_planner
{

DummyPlanner::DummyPlanner() : Node("DummyPlanner")
{
  RCLCPP_INFO(get_logger(), "Initializing DummyPlanner...");

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  task_server_ = std::make_unique<nav2_tasks::ComputePathToPoseTaskServer>(temp_node, false),
  task_server_->setExecuteCallback(
    std::bind(&DummyPlanner::computePathToPose, this, std::placeholders::_1));

  // Start listening for incoming ComputePathToPose task requests
  task_server_->startWorkerThread();

  RCLCPP_INFO(get_logger(), "Initialized DummyPlanner");
}

DummyPlanner::~DummyPlanner()
{
  RCLCPP_INFO(get_logger(), "Shutting down DummyPlanner");
}

TaskStatus
DummyPlanner::computePathToPose(const nav2_tasks::ComputePathToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
  "(%.2f, %.2f).", command->start.position.x, command->start.position.y,
  command->goal.position.x, command->goal.position.y);

  // Dummy path computation time
  std::this_thread::sleep_for(500ms);

  if (task_server_->cancelRequested()) {
    RCLCPP_INFO(get_logger(), "Cancelled planning task.");
    task_server_->setCanceled();
    return TaskStatus::CANCELED;
  }

  RCLCPP_INFO(get_logger(), "Found a dummy path");

  nav2_tasks::ComputePathToPoseResult result;
  task_server_->setResult(result);

  return TaskStatus::SUCCEEDED;
}

}  // namespace nav2_dummy_planner
