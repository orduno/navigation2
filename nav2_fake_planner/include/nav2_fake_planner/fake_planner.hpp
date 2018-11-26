// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
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

#ifndef NAV2_FAKE_PLANNER__FAKE_PLANNER_HPP_
#define NAV2_FAKE_PLANNER__FAKE_PLANNER_HPP_

#include <memory>

#include "nav2_tasks/compute_path_to_pose_task.hpp"

namespace nav2_fake_planner
{

class FakePlanner : public rclcpp::Node
{
public:
  FakePlanner();
  ~FakePlanner();

  nav2_tasks::TaskStatus computePathToPose(
    const nav2_tasks::ComputePathToPoseCommand::SharedPtr command);

private:
  std::unique_ptr<nav2_tasks::ComputePathToPoseTaskServer> task_server_;
};

}  // namespace nav2_fake_planner

#endif  // NAV2_FAKE_PLANNER__FAKE_PLANNER_HPP_
