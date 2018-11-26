// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
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
#include <ctime>
#include <thread>

#include "nav2_fake_controller/fake_controller.hpp"

using nav2_tasks::TaskStatus;
using namespace std::chrono_literals;

namespace nav2_fake_controller
{

FakeController::FakeController() : Node("FakeController")
{
  RCLCPP_INFO(get_logger(), "Initializing FakeController...");

  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](auto) {});

  vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("tb3/cmd_vel", 1);

  task_server_ = std::make_unique<nav2_tasks::FollowPathTaskServer>(temp_node, false),
  task_server_->setExecuteCallback(
    std::bind(&FakeController::followPath, this, std::placeholders::_1));

  // Start listening for incoming ComputePathToPose task requests
  task_server_->startWorkerThread();

  RCLCPP_INFO(get_logger(), "Initialized FakeController");
}

FakeController::~FakeController()
{
  RCLCPP_INFO(get_logger(), "Shutting down FakeController");
}

TaskStatus
FakeController::followPath(const nav2_tasks::FollowPathCommand::SharedPtr /*command*/)
{
  RCLCPP_INFO(get_logger(), "Starting controller ");

  auto start_time = std::chrono::system_clock::now();
  auto time_since_msg = std::chrono::system_clock::now();

  while (true) {
    // Fake controller computation time
    std::this_thread::sleep_for(50ms);

    // Log a message every second
    auto current_time = std::chrono::system_clock::now();
    if (current_time - time_since_msg >= 1s) {
      RCLCPP_INFO(get_logger(), "Following path");
      time_since_msg = std::chrono::system_clock::now();
    }

    // Output control command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.22;
    vel_pub_->publish(cmd_vel);

    if (task_server_->cancelRequested()) {
      RCLCPP_INFO(get_logger(), "Task cancelled");
      task_server_->setCanceled();
      return TaskStatus::CANCELED;
    }

    if (current_time - start_time >= 10s) {
      RCLCPP_INFO(get_logger(), "Reached end point");
      break;
    }
  }

  nav2_tasks::FollowPathResult result;
  task_server_->setResult(result);

  return TaskStatus::SUCCEEDED;
}

}  // namespace nav2_fake_controller
