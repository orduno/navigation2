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

#ifndef NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_
#define NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_

#include <string>
#include <QDialog>
#include <QBasicTimer>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/simple_action_client.hpp"
#include "rclcpp/rclcpp.hpp"

class QCheckBox;
class QDialogButtonBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPushButton;

class NavigationDialog : public QDialog
{
  Q_OBJECT

public:
  NavigationDialog(QWidget *parent = 0);

  void startNavigation(double x, double y, double theta, std::string & frame);

protected:
  void timerEvent(QTimerEvent *event);

private slots:
  void onCancelButtonPressed();

private:
  QLabel *label;
  QLineEdit *lineEdit;
  QCheckBox *fromStartCheckBox;
  QCheckBox *searchSelectionCheckBox;
  QDialogButtonBox *buttonBox;
  QPushButton *cancelButton;
  QPushButton *moreButton;
  QWidget *extension;

  QBasicTimer timer_;

  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<nav2_util::SimpleActionClient<nav2_msgs::action::NavigateToPose>> action_client_;
  nav2_msgs::action::NavigateToPose::Goal goal_;
};

#endif  //  NAV2_RVIZ_PLUGINS__NAVIGATION_DIALOG_HPP_
