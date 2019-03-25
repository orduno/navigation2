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

#include <memory>

#include "action_server.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

void execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");

  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Fibonacci::Feedback>();

  auto & sequence = feedback->sequence;
  sequence.push_back(0);
  sequence.push_back(1);

  auto result_response = std::make_shared<Fibonacci::Result>();

  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result_response->sequence = sequence;
      goal_handle->set_canceled(result_response);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
      return;
    }

    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);

    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result_response->sequence = sequence;
    goal_handle->set_succeeded(result_response);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Suceeded");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_server_node");
  auto action_server = std::make_shared<nav2_util::ActionServer<Fibonacci>>(node, "fibonacci");

  action_server->set_execute_callback(execute);

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
