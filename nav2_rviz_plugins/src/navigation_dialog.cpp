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

#include <string>
#include <QtWidgets>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_rviz_plugins/navigation_dialog.hpp"
#include "nav2_util/simple_action_client.hpp"

void
NavigationDialog::onCancelButtonPressed()
{
  action_client_->cancel();
}

NavigationDialog::NavigationDialog(QWidget *parent)
 : QDialog(parent)
{
  client_node_ = std::make_shared<rclcpp::Node>("nav_to_pose_client");
  action_client_ = std::make_shared<nav2_util::SimpleActionClient<nav2_msgs::action::NavigateToPose>>(client_node_, "NavigateToPose");
  goal_ = nav2_msgs::action::NavigateToPose::Goal();

  cancelButton = new QPushButton(tr("&Cancel"));
  cancelButton->setDefault(true);

  QHBoxLayout *layout = new QHBoxLayout;
  layout->addWidget(cancelButton);

  setMinimumWidth(120);
  setLayout(layout);
  setWindowTitle(tr("Navigating..."));
  setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);

  connect(cancelButton, SIGNAL(clicked()), this, SLOT(onCancelButtonPressed()));
}

void
NavigationDialog::timerEvent(QTimerEvent * event)
{
  if (event->timerId() == timer_.timerId()) {
    //update();
    auto result = action_client_->wait_for_result(std::chrono::milliseconds(1));

    switch (result) {
      case nav2_util::ActionStatus::SUCCEEDED:
      case nav2_util::ActionStatus::FAILED:
      case nav2_util::ActionStatus::CANCELED:
	    timer_.stop();
        hide();
        break;

      case nav2_util::ActionStatus::RUNNING:
        break;
    }
  } else {
    QWidget::timerEvent(event);
  }
}

geometry_msgs::msg::Quaternion
orientationAroundZAxis(double angle)
{
  auto orientation = geometry_msgs::msg::Quaternion();

  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = sin(angle) / (2 * cos(angle / 2));
  orientation.w = cos(angle / 2);

  return orientation;
}

void
NavigationDialog::startNavigation(double x, double y, double theta, std::string & frame)
{
  auto pose = std::make_shared<geometry_msgs::msg::PoseStamped>();

  pose->header.stamp = rclcpp::Clock().now();
  pose->header.frame_id = frame;
  pose->pose.position.x = x;
  pose->pose.position.y = y;
  pose->pose.position.z = 0.0;
  pose->pose.orientation = orientationAroundZAxis(theta);

  action_client_->wait_for_server();

  goal_.pose = *pose;
  action_client_->send_goal(goal_);

  timer_.start(100, this);
}

