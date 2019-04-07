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

#ifndef NAV2_RVIZ_PLUGINS__NAV_TOOL_HPP_
#define NAV2_RVIZ_PLUGINS__NAV_TOOL_HPP_

#include <atomic>
#include <memory>
#include <QObject>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

namespace rviz_common
{

class DisplayContext;

namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace nav2_rviz_plugins
{

class RVIZ_DEFAULT_PLUGINS_PUBLIC GoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  GoalTool();
  ~GoalTool() override;

  void activate() override;
  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  void onPoseSet(double x, double y, double theta) override;
  void invokeAction(double x, double y, double theta);

private:
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<nav2_util::SimpleActionClient<nav2_msgs::action::NavigateToPose>> action_client_;

  nav2_msgs::action::NavigateToPose::Goal goal_;

  std::atomic<bool> running_{false};
};

}  // namespace nav2_rviz_plugins

#endif  // NAV2_RVIZ_PLUGINS__NAV_TOOL_HPP_
