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
#include <inttypes.h>
#include <memory>

#include "simple_action_client.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using Goal = Fibonacci::Goal;

void feedback_callback(
  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
  const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  printf("Next number in sequence received: %d\n", feedback->sequence.back());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create our ROS node
  auto node = std::make_shared<rclcpp::Node>("test_client_node");

  // Create a SimpleActionClient
  auto action_client = std::make_shared<nav2_util::SimpleActionClient<Fibonacci>>(node, "fibonacci");

  for (;; ) {
    auto goal = Goal();
    goal.order = 10;

    printf("Press ENTER to send goal request..."); getchar();
    action_client->send_goal(goal, feedback_callback);

int i = 0;
    for (bool done=false; !done; ) {
      auto result = action_client->wait_for_result(std::chrono::milliseconds(250));
      switch (result)
      {
        case nav2_util::ActionStatus::SUCCEEDED:
            {
          RCLCPP_INFO(node->get_logger(), "Action succeeded");
              auto rc = action_client->get_result();

          for (auto number : rc.response->sequence) {
            printf("%d ", number);
          }
          printf("\n");
          done = true;
          break;
        }

        case nav2_util::ActionStatus::FAILED:
          RCLCPP_INFO(node->get_logger(), "Action failed");
          done = true;
          break;

        case nav2_util::ActionStatus::CANCELED:
          RCLCPP_INFO(node->get_logger(), "Action canceled");
          done = true;
          break;

        case nav2_util::ActionStatus::RUNNING:
          break;

        default:
          throw std::logic_error("Invalid status value");
      }

      i++;

      if (i == 10) {
       goal.order = 12;
       printf("i==5\n");
       //action_client->send_goal(goal, feedback_callback);
      }
    }

    //printf("Press ENTER to send cancel..."); getchar();
    //action_client->cancel();
  }

  rclcpp::shutdown();

  return 0;
}
