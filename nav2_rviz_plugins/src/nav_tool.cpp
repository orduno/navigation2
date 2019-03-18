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

#include "nav2_rviz_plugins/nav_tool.hpp"

#include <string>

#include "nav2_tasks/navigate_to_pose_task.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace nav2_rviz_plugins
{

NavTool::NavTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty("Topic", "goal",
      "The topic on which to publish navigation goals.",
      getPropertyContainer(), SLOT(updateTopic()), this);
}

NavTool::~NavTool() = default;

void NavTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Nav2 Tool");
  updateTopic();

  publisher_ = context_->getRosNodeAbstraction().lock()->get_raw_node()->create_publisher<geometry_msgs::msg::PoseStamped>("NavigateToPoseTask_command");
}

void NavTool::updateTopic()
{
}

void NavTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  auto goal = std::make_shared<geometry_msgs::msg::PoseStamped>();

  goal->header.stamp = rclcpp::Clock().now();
  goal->header.frame_id = fixed_frame;
  goal->pose.position.x = x;
  goal->pose.position.y = y;
  goal->pose.position.z = 0.0;
  goal->pose.orientation = orientationAroundZAxis(theta);

  publisher_->publish(goal);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::NavTool, rviz_common::Tool)
