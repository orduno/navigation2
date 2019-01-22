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

#ifndef NAV2_SIMPLE_PLANNER__WORLD_HPP_
#define NAV2_SIMPLE_PLANNER__WORLD_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_tasks/costmap_service_client.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"

namespace nav2_simple_planner
{

class World
{
public:
  explicit World(rclcpp::Node::SharedPtr & node);

  World() = delete;

  void update();

  // Transform a point from world to map frame
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my);

  // Transform a point from map to world frame
  void mapToWorld(double mx, double my, double & wx, double & wy);

  bool isOccupiedCell(const unsigned int mx, const unsigned my);

  bool isEmpty();

  bool isWithinBounds(const unsigned int mx, const unsigned int my);

  // TODO(orduno)
  // unsigned char getCost(const unsigned int mx, const unsigned int my);

  unsigned int numCellsX() { return costmap_.metadata.size_x; }

  unsigned int numCellsY() { return costmap_.metadata.size_y; }

private:
  // The ROS node to use to create publishers and subscribers
  rclcpp::Node::SharedPtr node_;

  // Using a costmap to represent the world
  nav2_msgs::msg::Costmap costmap_;
  bool hasCostmap_;

  // TODO(orduno) remove after testing
  OccupancyGrid occupancy_;

  // Service client for getting the costmap
  nav2_tasks::CostmapServiceClient costmap_client_;

  // Request costmap from world model
  void getCostmap(
    nav2_msgs::msg::Costmap & costmap, const std::string layer = "master",
    const std::chrono::milliseconds waitTime = std::chrono::milliseconds(100));
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__WORLD_HPP_
