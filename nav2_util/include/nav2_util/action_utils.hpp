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

#ifndef NAV2_UTIL__ACTION_UTILS_HPP_
#define NAV2_UTIL__ACTION_UTILS_HPP_

#include <vector>
#include <string>
#include <chrono>

#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_util
{

typedef enum { SUCCEEDED, FAILED, CANCELED } ActionStatus;

template<typename ActionT>
ActionStatus invoke(
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_,
  const typename ActionT::Goal & goal,
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result,
  typename rclcpp_action::ClientGoalHandle<ActionT>::FeedbackCallback callback = nullptr)
{
  action_client_->wait_for_action_server();

  // Send the goal
  auto future_goal_handle = action_client_->async_send_goal(goal, callback);
  if (rclcpp::spin_until_future_complete(node_, future_goal_handle) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Send goal failed");
    return FAILED;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_INFO(node_->get_logger(), "Goal was rejected by the action server");
    return FAILED;
  }

  // Wait for the result
  auto future_result = action_client_->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node_, future_result) !=
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node_->get_logger(), "Failed to get the future result");
    return FAILED;
  }

  result = future_result.get();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return SUCCEEDED;

    case rclcpp_action::ResultCode::ABORTED:
      return FAILED;

    case rclcpp_action::ResultCode::CANCELED:
      return CANCELED;

    default:
      throw std::runtime_error("Unknown result code");
  }
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__ACTION_UTILS_HPP_
