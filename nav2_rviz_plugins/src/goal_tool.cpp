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

#include "nav2_rviz_plugins/goal_tool.hpp"

#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_client.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "/home/mjeronimo/src/ros2/src/ros2/rviz/rviz_common/src/rviz_common/tool_manager.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace nav2_rviz_plugins
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
}

GoalTool::~GoalTool() = default;

void GoalTool::onInitialize()
{
  PoseTool::onInitialize();

  setName("2D Nav2 Goal");
  setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/SetGoal.png"));

  client_node_ = std::make_shared<rclcpp::Node>("nav_to_pose_client");
  action_client_ = std::make_shared<nav2_util::SimpleActionClient<nav2_msgs::action::NavigateToPose>>(client_node_, "navigate_to_pose");

  goal_ = nav2_msgs::action::NavigateToPose::Goal();
}

void GoalTool::invokeAction(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  pose->header.stamp = rclcpp::Clock().now();
  pose->header.frame_id = fixed_frame;
  pose->pose.position.x = x;
  pose->pose.position.y = y;
  pose->pose.position.z = 0.0;
  pose->pose.orientation = orientationAroundZAxis(theta);

  action_client_->wait_for_server();

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult result;
  goal_.pose = *pose;

  running_ = true;
  /*auto status =*/ action_client_->invoke(goal_, result);
  running_ = false;

  setName("2D Nav2 Goal");
  setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/SetGoal.png"));
  context_->getToolManager()->refreshTool(this);
}

void GoalTool::activate()
{
  setStatus("Foobar: Click and drag mouse to set position/orientation.");
  state_ = Position;
}

int GoalTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftDown() && running_) {
    printf("Execute Cancel***********************************\n");

    setName("2D Nav2 Goal");
    setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/SetGoal.png"));
    context_->getToolManager()->refreshTool(this);

	running_ = false;
    return 0;
  }

  return PoseTool::processMouseEvent(event);
}

static std::future<void> future_result;

void GoalTool::onPoseSet(double x, double y, double theta)
{
  setName("Cancel Navigation");
  setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/Time.svg"));
  context_->getToolManager()->refreshTool(this);

  future_result = std::async([this](double x, double y, double theta) -> void { invokeAction(x, y, theta); }, x, y, theta);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::GoalTool, rviz_common::Tool)
