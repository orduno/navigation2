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

#ifndef NAV2_TASKS__BT_ACTION_NODE_HPP_
#define NAV2_TASKS__BT_ACTION_NODE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "nav2_util/simple_action_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_tasks
{

template<class ActionT>
class BtActionNode : public BT::CoroActionNode
{
public:
  explicit BtActionNode(const std::string & action_name)
  : BT::CoroActionNode(action_name), action_name_(action_name), action_client_(nullptr)
  {
  }

  BtActionNode(const std::string & action_name, const BT::NodeParameters & params)
  : BT::CoroActionNode(action_name, params), action_name_(action_name), action_client_(nullptr)
  {
  }

  BtActionNode() = delete;

  virtual ~BtActionNode()
  {
  }

  // This is a callback from the BT library invoked after the node is created and after the
  // blackboard has been set for the node. It is the first opportunity for the node to access
  // the blackboard. The derived class does not override this method, but overrides onConfigure
  void onInit() final
  {
    // Get the required items from the blackboard
    node_ = blackboard()->template get<rclcpp::Node::SharedPtr>("node");
    node_loop_timeout_ =
      blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

    // Now that we have the ROS node to use, create the action client for this BT action
    action_client_ = std::make_unique<nav2_util::SimpleActionClient<ActionT>>(node_, action_name_);

    // Make sure the server is actually there before continuing
    action_client_->wait_for_server();

    // Give the derived class a chance to do some initialization
    onConfigure();
  }

  // Derived classes can override this method to perform some local initialization such
  // as getting values from the blackboard.
  virtual void onConfigure()
  {
  }

  BT::NodeStatus tick() override
  {
    action_client_->send_goal(goal_);
    for (;; ) {
      auto rc = action_client_->wait_for_result(node_loop_timeout_);

      switch (rc) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
		      result_ = action_client_->get_result();
          setStatus(BT::NodeStatus::IDLE);
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          setStatus(BT::NodeStatus::IDLE);
          return BT::NodeStatus::FAILURE;

        case nav2_tasks::TaskStatus::CANCELED:
          setStatus(BT::NodeStatus::IDLE);
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::RUNNING:
          setStatusRunningAndYield();
          break;

        default:
          throw std::logic_error("BtActionNode::Tick: invalid status value");
      }
    }
  }

  void halt() override
  {
    // Shut the node down if it is currently running
    if (status() == BT::NodeStatus::RUNNING) {
      action_client_->cancel();
    }

    CoroActionNode::halt();
  }

protected:
  const std::string & action_name_;
  typename std::unique_ptr<nav2_util::SimpleActionClient<ActionT>> action_client_;

  typename ActionT::Goal goal_;
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds node_loop_timeout_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
