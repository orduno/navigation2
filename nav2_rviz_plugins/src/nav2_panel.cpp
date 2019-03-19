// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QtConcurrent/QtConcurrent>

#include "rviz_common/display_context.hpp"

using namespace QtConcurrent;

namespace nav2_rviz_plugins
{

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent)
{
  QPushButton * startup_button = new QPushButton("Startup");
  QPushButton * shutdown_button = new QPushButton("Shutdown");
  QPushButton * cancel_button = new QPushButton("Cancel Navigation");

  connect(startup_button, SIGNAL(clicked()), this, SLOT(onStartupClicked()));
  connect(shutdown_button, SIGNAL(clicked()), this, SLOT(onShutdownClicked()));
  connect(cancel_button, SIGNAL(clicked()), this, SLOT(onCancelClicked()));

  startup_button->setToolTip("TODO");
  shutdown_button->setToolTip("TODO");
  cancel_button->setToolTip("TODO");

  QHBoxLayout * top_layout = new QHBoxLayout;
  top_layout->addWidget(cancel_button);
  top_layout->setContentsMargins(2, 6, 2, 2);

  QHBoxLayout * button_layout = new QHBoxLayout;
  button_layout->addWidget(startup_button);
  button_layout->addWidget(shutdown_button);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(0, 0, 0, 0);
  main_layout->addLayout(top_layout);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  cancel_pub_ = node->create_publisher<std_msgs::msg::Empty>("NavigateToPoseTask_cancel");
}

void
Nav2Panel::onStartupClicked()
{
  QFuture<void> future = QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::startup, &client_));
  // TODO: start a timer to check on result(?)
}

void
Nav2Panel::onShutdownClicked()
{
  QFuture<void> future = QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::shutdown, &client_));
  // TODO: start a timer to check on result(?)
}

void
Nav2Panel::onCancelClicked()
{
  auto msg = std::make_shared<std_msgs::msg::Empty>();
  cancel_pub_->publish(msg);
}

void
Nav2Panel::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void
Nav2Panel::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::Nav2Panel, rviz_common::Panel)
