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

#ifndef NAV2_UTIL__ACTION_CLIENT_HPP_
#define NAV2_UTIL__ACTION_CLIENT_HPP_

#include <chrono>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

template<typename ActionT>
class ActionClient
{
public:
  explicit ActionClient(rclcpp::Node::SharedPtr node, const std::string & action_name)
  : node_(node)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node, action_name);
  }

  void send_goal(const typename ActionT::GoalRequestService::Request & goal)
  {
    auto future_goal_handle = action_client_->async_send_goal(goal
        /*FeedbackCallback callback = nullptr, bool ignore_result = false*/);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      throw std::runtime_error("send goal call failed");
    }

    goal_handle_ = future_goal_handle.get();

    if (!goal_handle_) {
      throw std::runtime_error("Goal was rejected by the server");
    }
  }

  bool cancel()
  {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    return rclcpp::spin_until_future_complete(node_, future_cancel) ==
           rclcpp::executor::FutureReturnCode::SUCCESS;
  }

  void send_goal_update(const typename ActionT::GoalRequestService::Request & goal)
  {
    cancel();
    send_goal(goal);
  }

  void cancel_all_goals()
  {
    // async_cancel_all_goals
  }

  bool wait_for_server(std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    return action_client_->wait_for_action_server(timeout);
  }

  bool wait_for_result(std::chrono::milliseconds timeout = std::chrono::milliseconds::max())
  {
    auto future_result = action_client_->async_get_result(goal_handle_);
    auto wait_result = rclcpp::spin_until_future_complete(node_, future_result, timeout);

    if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT) {
      return false;
    } else if (wait_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("get result failed");
    }

    auto result = future_result.get();

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
        return;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
        return;

      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
        return;
    }
  }

protected:
  // The ActionClient isn't itself a node, so needs to know which one to use
  rclcpp::Node::SharedPtr node_;

  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;

  typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle_;
};

}  // namespace nav2_util

#endif   // NAV2_UTIL__ACTION_CLIENT_HPP_
