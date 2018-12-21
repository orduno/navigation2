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

#ifndef NAV2_WORLD_MODEL__COSTMAP_REPRESENTATION_HPP_
#define NAV2_WORLD_MODEL__COSTMAP_REPRESENTATION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_world_model/world_representation.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/transform_listener.h"

namespace nav2_world_model
{

class CostmapRepresentation : public WorldRepresentation
{
public:
  CostmapRepresentation(
    const std::string & name,
    rclcpp::executor::Executor & executor,
    rclcpp::Clock::SharedPtr & clock);

  GetCostmap::Response getCostmap(const GetCostmap::Request & request) override;

  ProcessRegion::Response confirmFreeSpace(const ProcessRegion::Request & request) override;

  ProcessRegion::Response clearArea(const ProcessRegion::Request & request) override;

private:
  std::string name_;

  rclcpp::Clock::SharedPtr & clock_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  // The implementation of the costmap
  // TODO(orduno) should really be shared unique?
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // TODO(orduno) this should be a shared ptr
  nav2_costmap_2d::Costmap2D * costmap_;
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__COSTMAP_REPRESENTATION_HPP_
