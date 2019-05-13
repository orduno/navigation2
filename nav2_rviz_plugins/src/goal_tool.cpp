// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/window_manager_interface.hpp"

namespace nav2_rviz_plugins
{

GoalTool::GoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';

  navigation_dialog_ = std::make_unique<NavigationDialog>();
  navigation_dialog_->move(0, 0);
}

GoalTool::~GoalTool()
{
}

void
GoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Navigation2 Goal");
  setIcon(rviz_common::loadPixmap("package://nav2_rviz_plugins/icons/SetGoal.png"));
}

int
GoalTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftUp()) {
    x_ = event.x;
    y_ = event.y;
  }

  return PoseTool::processMouseEvent(event);
}

void
GoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  auto parent = context_->getWindowManager()->getParentWindow();
  auto global_pos = parent->mapToGlobal(QPoint(x_,y_));

  navigation_dialog_->move(global_pos);
  navigation_dialog_->startNavigation(x, y, theta, fixed_frame);
  navigation_dialog_->show();
  navigation_dialog_->raise();
  navigation_dialog_->activateWindow();
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::GoalTool, rviz_common::Tool)
