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

#include "nav2_simple_planner/world.hpp"

namespace nav2_simple_planner
{

World::World(rclcpp::Node::SharedPtr & node)
: node_(node),
  hasCostmap_(false)
{
}

void
World::update()
{
  getCostmap(costmap_);
  hasCostmap_ = true;
}

void World::getCostmap(
  nav2_msgs::msg::Costmap & costmap, const std::string /*layer*/,
  const std::chrono::milliseconds /*waitTime*/)
{
  // TODO(orduno): explicitly provide specifications for costmap using the costmap on the request,
  //               including master (aggregate) layer

  auto request = std::make_shared<nav2_tasks::CostmapServiceClient::CostmapServiceRequest>();
  request->specs.resolution = 1.0;

  auto result = costmap_client_.invoke(request);
  costmap = result.get()->map;

  RCLCPP_DEBUG(node_->get_logger(), "Costmap size: %d, %d",
    costmap_.metadata.size_x, costmap_.metadata.size_y);
}

bool World::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_.metadata.origin.position.x || wy < costmap_.metadata.origin.position.y) {
    RCLCPP_ERROR(node_->get_logger(), "wordToMap failed: "
      "wx, wy: %f, %f, size_x, size_y: %d, %d", wx, wy,
      costmap_.metadata.size_x, costmap_.metadata.size_y);
    return false;
  }

  mx = static_cast<unsigned int>(
    std::round((wx - costmap_.metadata.origin.position.x) / costmap_.metadata.resolution));
  my = static_cast<unsigned int>(
    std::round((wy - costmap_.metadata.origin.position.y) / costmap_.metadata.resolution));

  if (mx < costmap_.metadata.size_x && my < costmap_.metadata.size_y) {
    return true;
  }

  RCLCPP_ERROR(node_->get_logger(), "wordToMap failed: "
    "mx, my: %d, %d, size_x, size_y: %d, %d", mx, my,
    costmap_.metadata.size_x, costmap_.metadata.size_y);

  return false;
}

void World::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_.metadata.origin.position.x + mx * costmap_.metadata.resolution;
  wy = costmap_.metadata.origin.position.y + my * costmap_.metadata.resolution;
}

bool World::isOccupiedCell(const unsigned int mx, const unsigned my)
{
  unsigned int index = my * costmap_.metadata.size_x + mx;
  return isOccupiedCell(index);
}

bool World::isOccupiedCell(const unsigned int index)
{
  // TODO(orduno)
  CostValue inscribed_inflated_obstacle = 253;

  if (costmap_.data[index] < inscribed_inflated_obstacle) {
    return false;
  }

  return true;
}

bool World::isEmpty()
{
  if (hasCostmap_) {
    return costmap_.data.empty();
  }
  return true;
}

bool
World::isWithinBounds(const unsigned int mx, const unsigned int my)
{
  return !(my >= numCellsY() || mx >= numCellsX());
}

}  // namespace nav2_simple_planner
