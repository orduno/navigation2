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

#ifndef NAV2_UTIL__RATE_CONSTRAINT_HPP_
#define NAV2_UTIL__RATE_CONSTRAINT_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_util
{

class RateConstraint
{
public:
  RateConstraint(
    std::string constraint_id, uint32_t target_rate, uint32_t jitter_margin,
    std::function<void(int iter_num, rclcpp::Duration actual)> violation_cb = nullptr);
  RateConstraint() = delete;
  ~RateConstraint();

  void calc_looptime();

protected:
  void print_duration(rclcpp::Duration dur);
  void print_metrics();

  uint32_t iter_cnt_{0};
  uint32_t rate_{0};
  uint32_t jitter_margin_{0};

  rclcpp::Time start_{0, 0};
  rclcpp::Time prev_looptime_{0, 0};
  rclcpp::Duration acceptable_looptime_{0, 0};

  std::function<void(int, rclcpp::Duration)> overrun_cb_;

  FILE * log_file_;
  bool first_time_{true};
  rclcpp::Clock clock_;
};

}  // namespace nav2_util

#endif  // NAV2_UTIL__RATE_CONSTRAINT_HPP_
