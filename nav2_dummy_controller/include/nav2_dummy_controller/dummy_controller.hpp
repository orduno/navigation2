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

#ifndef NAV2_DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_
#define NAV2_DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_

#include <memory>

#include "nav2_tasks/follow_path_task.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace nav2_dummy_controller
{

class DummyController : public rclcpp::Node
{
public:
  DummyController();
  ~DummyController();

  nav2_tasks::TaskStatus followPath(
    const nav2_tasks::FollowPathCommand::SharedPtr command);

private:
  void setZeroVelocity();

  std::unique_ptr<nav2_tasks::FollowPathTaskServer> task_server_;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist>> vel_pub_;
};

}  // namespace nav2_dummy_controller

#endif  // NAV2_DUMMY_CONTROLLER__DUMMY_CONTROLLER_HPP_
