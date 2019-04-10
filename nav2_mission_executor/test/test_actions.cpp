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

//#include "gtest/gtest.h"
#include "nav2_util/simple_action_client.hpp"
#include "nav2_msgs/action/execute_mission.hpp"
#include "rclcpp/rclcpp.hpp"

static const std::string xml_text = R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <NavigateToPose position="1;2;0" orientation="0;0;0;1"/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("mission_executor_test_node");
  auto action_client = std::make_shared<nav2_util::SimpleActionClient<nav2_msgs::action::ExecuteMission>>(node, "ExecuteMission");

  action_client->wait_for_server();

  auto goal = nav2_msgs::action::ExecuteMission::Goal();
  goal.mission_plan.mission_plan = xml_text;

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::ExecuteMission>::WrappedResult result;

  auto status = action_client->invoke(goal, result);
  RCLCPP_INFO(node->get_logger(), "status: %d", status);

  rclcpp::shutdown();
  return 0;
}
