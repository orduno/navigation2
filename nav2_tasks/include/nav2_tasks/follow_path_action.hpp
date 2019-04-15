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

#ifndef NAV2_TASKS__FOLLOW_PATH_ACTION_HPP_
#define NAV2_TASKS__FOLLOW_PATH_ACTION_HPP_

#include <string>
#include <memory>

#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_tasks/bt_action_node.hpp"

namespace nav2_tasks
{

class FollowPathAction : public BtActionNode<nav2_msgs::action::FollowPath>
{
public:
  explicit FollowPathAction(const std::string & action_name)
  : BtActionNode<nav2_msgs::action::FollowPath>(action_name)
  {
  }

  void on_tick() override
  {
	goal_.path = *(blackboard()->get<nav2_msgs::msg::Path::SharedPtr>("path"));
    // blackboard()->set<bool>("goal_reached", false);
  }

  void on_loop_timeout() override
  {
    // Check of the goal has been updated
	if (blackboard()->get<bool>("path_updated")) {
	  goal_.path = *(blackboard()->get<nav2_msgs::msg::Path::SharedPtr>("path"));
	  blackboard()->set<bool>("path_updated", false);
      // action_client_->send_goal(goal_);
	}
  }

  void on_success() override
  {
    blackboard()->set<bool>("goal_reached", true);
  }

};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__FOLLOW_PATH_ACTION_HPP_
