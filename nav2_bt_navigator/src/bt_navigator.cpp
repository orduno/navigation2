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

#include <string>
#include <memory>
#include <sstream>
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_bt_navigator/navigate_to_pose_behavior_tree.hpp"
#include "nav2_tasks/compute_path_to_pose_task.hpp"
#include "nav2_tasks/bt_conversions.hpp"
#include "Blackboard/blackboard_local.h"

using nav2_tasks::TaskStatus;

namespace nav2_bt_navigator
{

BtNavigator::BtNavigator()
: Node("NavigateToPoseNode")
{
  auto temp_node = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});

  robot_ = std::make_unique<nav2_robot::Robot>(temp_node);

  task_server_ = std::make_unique<nav2_tasks::NavigateToPoseTaskServer>(temp_node);
  task_server_->setExecuteCallback(
    std::bind(&BtNavigator::navigateToPose, this, std::placeholders::_1));
}

TaskStatus
BtNavigator::navigateToPose(const nav2_tasks::NavigateToPoseCommand::SharedPtr command)
{
  RCLCPP_INFO(get_logger(), "Start navigating to goal (%.2f, %.2f).",
    command->pose.position.x, command->pose.position.y);

  // Get the current pose from the robot
  auto current = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

  // if (!robot_->getCurrentPose(current)) {
  //   RCLCPP_ERROR(get_logger(), "Current robot pose is not available.");
  //   return TaskStatus::FAILED;
  // }

  // Create the blackboard that will be shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard = BT::Blackboard::create<BT::BlackboardLocal>();

  // Put together the PathEndPoints message for the ComputePathToPose
  auto endpoints = std::make_shared<nav2_tasks::ComputePathToPoseCommand>();
  endpoints->start = current->pose.pose;
  endpoints->goal = command->pose;
  endpoints->tolerance = 2.0;  // TODO(mjeronimo): this will come in the command message

  // The path returned from ComputePath and sent to FollowPath
  auto path = std::make_shared<nav2_tasks::ComputePathToPoseResult>();

  // Set the shared data (commands/results)
  blackboard->set<nav2_tasks::ComputePathToPoseCommand::SharedPtr>("endpoints", endpoints);
  blackboard->set<nav2_tasks::ComputePathToPoseResult::SharedPtr>("path", path);  // NOLINT

  std::string xml_text =
    R"(
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="root">
      <Sequence name="check_preconditions">
        <Fallback name="check_motion">
          <Inverter name="is_stuck">
            <IsStuck/>
          </Inverter>
          <Sequence name="stuck_recovery">
            <Stop/>
            <Spin/>
          </Sequence>
          <Spin/>
        </Fallback>
      </Sequence>
      <SequenceStar name="navigate">
        <ComputePathToPose endpoints="${endpoints}" path="${path}"/>
        <FollowPath path="${path}"/>
      </SequenceStar>
    </Sequence>
  </BehaviorTree>
</root>)";

  RCLCPP_INFO(get_logger(), "Behavior tree XML: %s", xml_text.c_str());

  // Create and run the behavior tree
  NavigateToPoseBehaviorTree bt(shared_from_this());
  TaskStatus result = bt.run(blackboard, xml_text,
      std::bind(&nav2_tasks::NavigateToPoseTaskServer::cancelRequested, task_server_.get()));

  RCLCPP_INFO(get_logger(), "Completed navigation: result: %d", result);
  return result;
}

}  // namespace nav2_bt_navigator
