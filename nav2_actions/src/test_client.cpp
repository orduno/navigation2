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

#include "action_client.hpp"
#include "test_msgs/action/fibonacci.hpp"

using Fibonacci = test_msgs::action::Fibonacci;
using Goal = Fibonacci::Goal;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_client_node");
  auto action_client = std::make_shared<nav2_util::ActionClient<Fibonacci>>(node, "fibonacci");

  for (;; ) {
    auto goal = Goal();
    goal.order = 10;

    printf("Press ENTER to send goal request..."); getchar();
    action_client->send_goal(goal);

    // printf("Press ENTER to get result..."); getchar();
    // action_client->wait_for_result();

    printf("Press ENTER to send goal request..."); getchar();
    action_client->send_goal(goal);

    printf("Press ENTER to send cancel..."); getchar();
    action_client->cancel();
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
