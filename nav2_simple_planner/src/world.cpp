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
  hasCostmap_(false),
  occupancy_(
    OccupancyGrid {
    // 0  1  2  3  4  5  6  7  8  9
      {0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // 0
      {0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, // 1
      {1, 0, 1, 0, 1, 0, 1, 0, 0, 1}, // 2
      {0, 0, 1, 0, 1, 0, 1, 1, 0, 0}, // 3
      {0, 0, 1, 0, 1, 0, 0, 1, 0, 0}, // 4
      {0, 0, 1, 0, 1, 1, 0, 0, 1, 0}, // 5
      {0, 0, 0, 0, 0, 0, 1, 0, 1, 0}, // 6
      {0, 0, 1, 0, 1, 0, 0, 0, 1, 0}, // 7
      {0, 0, 1, 0, 1, 1, 1, 1, 1, 0}, // 8
      {0, 0, 1, 0, 0, 0, 0, 0, 1, 0}  // 9
  })
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

  // auto request = std::make_shared<nav2_tasks::CostmapServiceClient::CostmapServiceRequest>();
  // request->specs.resolution = 1.0;

  // auto result = costmap_client_.invoke(request);
  // costmap = result.get()->map;

  // RCLCPP_DEBUG(node_->get_logger(), "Costmap size: %d, %d",
    // costmap_.metadata.size_x, costmap_.metadata.size_y);

  // TODO(orduno) remove, added just for testing
  costmap.metadata.origin.position.x = 0.0;
  costmap.metadata.origin.position.y = 0.0;
  costmap.metadata.resolution = 1.0;
  costmap.metadata.size_x = 10.0;
  costmap.metadata.size_y = 10.0;
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
  return occupancy_.isOccupied(Point{static_cast<int>(my), static_cast<int>(mx)});
}

bool World::isEmpty()
{
  if (hasCostmap_) {
    // return costmap_.data.empty();
    return false;
  }
  return true;
}

bool
World::isWithinBounds(const unsigned int mx, const unsigned int my)
{
  return !(my >= numCellsY() || mx >= numCellsX());
}

}  // namespace nav2_simple_planner
