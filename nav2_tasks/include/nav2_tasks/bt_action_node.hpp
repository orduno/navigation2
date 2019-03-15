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
#include <condition_variable>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_tasks/task_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_tasks
{

template<class CommandMsg, class ResultMsg>
class BtActionNode : public BT::CoroActionNode
{
public:
  explicit BtActionNode(const std::string & action_name)
  : BT::CoroActionNode(action_name), task_client_(nullptr)
  {
  }

  BtActionNode(const std::string & action_name, const BT::NodeParameters & params)
  : BT::CoroActionNode(action_name, params), task_client_(nullptr)
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
    node_ = blackboard()->template get<nav2_lifecycle::LifecycleNode::SharedPtr>("node");
    node_loop_timeout_ =
      blackboard()->template get<std::chrono::milliseconds>("node_loop_timeout");

    // Now that we have the ROS node to use, create the task client for this action
    //
    // TODO(mjeronimo): There is not currently a way for Behavior Trees to track with the
    // lifecycle states. So, instance the task client here, having it automatically configure
    // and activate.
    //
    task_client_ = std::make_unique<nav2_tasks::TaskClient<CommandMsg, ResultMsg>>(node_, true);

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
    task_client_->sendCommand(command_);

    // Loop until the task has completed
    for (;; ) {
      nav2_tasks::TaskStatus status = task_client_->waitForResult(result_, node_loop_timeout_);

      switch (status) {
        case nav2_tasks::TaskStatus::SUCCEEDED:
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::FAILED:
          return BT::NodeStatus::FAILURE;

        case nav2_tasks::TaskStatus::CANCELED:
          cv_cancel_.notify_one();
          return BT::NodeStatus::SUCCESS;

        case nav2_tasks::TaskStatus::RUNNING:
          setStatusRunningAndYield();
          break;

        default:
          throw std::logic_error("BtActionNode::Tick: invalid status value");
      }
    }

    // Should never get here. Return statement added to avoid compiler warning.
    return BT::NodeStatus::SUCCESS;
  }

  void halt() override
  {
    // Send a cancel message to the task server
    task_client_->cancel();

    // Then wait for the response before continuing
    std::unique_lock<std::mutex> lock(cancel_mutex_);
    cv_cancel_.wait(lock);

    CoroActionNode::halt();
  }

protected:
  typename std::unique_ptr<nav2_tasks::TaskClient<CommandMsg, ResultMsg>> task_client_;

  // The node that will be used for any ROS operations
  nav2_lifecycle::LifecycleNode::SharedPtr node_;

  // The timeout value while to use in the tick loop while waiting for
  // a result from the server
  std::chrono::milliseconds node_loop_timeout_;

  typename CommandMsg::SharedPtr command_;
  typename ResultMsg::SharedPtr result_;

  // Allow for signaling receipt of the cancel message
  std::mutex cancel_mutex_;
  std::condition_variable cv_cancel_;
};

}  // namespace nav2_tasks

#endif  // NAV2_TASKS__BT_ACTION_NODE_HPP_
