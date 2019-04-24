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

#include "nav2_rviz_plugins/nav2_panel.hpp"

#include <dirent.h>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidgetItem>
#include <QPushButton>
#include <QtConcurrent/QtConcurrent>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"

namespace nav2_rviz_plugins
{

Nav2Panel::Nav2Panel(QWidget * parent)
: Panel(parent)
{
  // Create the control button and its tooltip

  start_stop_button_ = new QPushButton;
  pause_resume_button_ = new QPushButton;

  // Create the state machine used to present the proper control button states in the UI

  const char * startup_msg = "Configure and activate all nav2 lifecycle nodes";
  const char * shutdown_msg1 = "Deactivate and cleanup all nav2 lifecycle nodes";
  const char * shutdown_msg2 = "Deactivate, cleanup, and shutdown all nav2 lifecycle nodes";

  initial_ = new QState();
  initial_->setObjectName("initial");
  initial_->assignProperty(start_stop_button_, "text", "Startup");
  initial_->assignProperty(start_stop_button_, "toolTip", startup_msg);
  initial_->assignProperty(pause_resume_button_, "text", "Pause");
  initial_->assignProperty(pause_resume_button_, "enabled", false);
  initial_->assignProperty(pause_resume_button_, "toolTip", startup_msg);

  starting_ = new QState();
  starting_->setObjectName("starting");
  starting_->assignProperty(start_stop_button_, "text", "Shutdown");
  starting_->assignProperty(start_stop_button_, "toolTip", shutdown_msg2);
  starting_->assignProperty(pause_resume_button_, "enabled", true);
  starting_->assignProperty(pause_resume_button_, "toolTip", shutdown_msg1);

  stopping_ = new QState();
  stopping_->setObjectName("stopping");
  stopping_->assignProperty(start_stop_button_, "enabled", false);
  stopping_->assignProperty(pause_resume_button_, "enabled", false);

  pausing_ = new QState();
  pausing_->setObjectName("pausing");
  pausing_->assignProperty(start_stop_button_, "enabled", false);
  pausing_->assignProperty(pause_resume_button_, "text", "Resume");
  pausing_->assignProperty(pause_resume_button_, "toolTip", startup_msg);

  resuming_ = new QState();
  resuming_->setObjectName("resuming");
  resuming_->assignProperty(start_stop_button_, "enabled", true);
  resuming_->assignProperty(pause_resume_button_, "text", "Pause");
  resuming_->assignProperty(pause_resume_button_, "toolTip", shutdown_msg1);

  QObject::connect(starting_, SIGNAL(entered()), this, SLOT(onStartup()));
  QObject::connect(stopping_, SIGNAL(entered()), this, SLOT(onShutdown()));
  QObject::connect(pausing_, SIGNAL(entered()), this, SLOT(onPause()));
  QObject::connect(resuming_, SIGNAL(entered()), this, SLOT(onResume()));

  initial_->addTransition(start_stop_button_, SIGNAL(clicked()), starting_);
  starting_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);
  starting_->addTransition(pause_resume_button_, SIGNAL(clicked()), pausing_);
  pausing_->addTransition(pause_resume_button_, SIGNAL(clicked()), resuming_);
  resuming_->addTransition(pause_resume_button_, SIGNAL(clicked()), pausing_);
  resuming_->addTransition(start_stop_button_, SIGNAL(clicked()), stopping_);

  machine_.addState(initial_);
  machine_.addState(starting_);
  machine_.addState(stopping_);
  machine_.addState(pausing_);
  machine_.addState(resuming_);

  machine_.setInitialState(initial_);
  machine_.start();

  // Display the available log files

  QLabel *label = new QLabel(this);
  label->setText("Rate Monitors:");

  listWidget_ = new QListWidget(this);
  listWidget_->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(listWidget_, SIGNAL(itemDoubleClicked(QListWidgetItem *)),
    this, SLOT(itemDoubleClicked(QListWidgetItem *)));

  loadLogFiles();

  // Lay out the items in the panel

  QVBoxLayout * button_layout = new QVBoxLayout;
  button_layout->addWidget(pause_resume_button_);
  button_layout->addWidget(start_stop_button_);
  button_layout->setContentsMargins(2, 0, 2, 2);

  QVBoxLayout * main_layout = new QVBoxLayout;
  main_layout->setContentsMargins(10, 10, 10, 10);
  main_layout->addWidget(label);
  main_layout->addWidget(listWidget_);
  main_layout->addLayout(button_layout);
  setLayout(main_layout);
}

void
Nav2Panel::itemDoubleClicked(QListWidgetItem * list_item)
{
  std::string filename(list_item->text().toStdString());
  std::string cmd("python3 ~/src/navigation2/nav2_util/src/real_time/plot_bar.py -f /tmp/nav2/");
  cmd += filename + ".log";
  int rc = system(cmd.c_str());
  (void)rc;
}

void
Nav2Panel::loadLogFiles()
{
  listWidget_->clear();

  DIR *dir;
  if ((dir = opendir ("/tmp/nav2")) != nullptr) {
    struct dirent *ent;
    while ((ent = readdir (dir)) != nullptr) {
	  if (ent->d_name[0] != '.') {

	    std::string filename(ent->d_name);

		size_t lastindex = filename.find_last_of(".");
        std::string basename = filename.substr(0, lastindex);

		QList<QListWidgetItem *> items = listWidget_->findItems(basename.c_str(), Qt::MatchExactly);
        if (items.size() == 0) {
          new QListWidgetItem(tr(basename.c_str()), listWidget_);
        }
      }
    }
    closedir (dir);
    listWidget_->sortItems(Qt::AscendingOrder);
  }
}

void
Nav2Panel::onInitialize()
{
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
}

void
Nav2Panel::onStartup()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::startup, &client_));
}

void
Nav2Panel::onShutdown()
{
  loadLogFiles();

  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::shutdown, &client_));
}

void
Nav2Panel::onPause()
{
  loadLogFiles();

  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::pause, &client_));
}

void
Nav2Panel::onResume()
{
  QFuture<void> future =
    QtConcurrent::run(std::bind(&nav2_controller::Nav2ControllerClient::resume, &client_));
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
