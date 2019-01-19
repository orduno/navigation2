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

#ifndef NAV2_SIMPLE_PLANNER__FULL_MOTION_HPP_
#define NAV2_SIMPLE_PLANNER__FULL_MOTION_HPP_

#include <vector>
#include <string>
#include <memory>

#include "nav2_simple_planner/motion.hpp"
#include "nav2_simple_planner/point.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"

namespace nav2_simple_planner
{

class FullMotion : public Motion
{
public:

  FullMotion()
  : Motion()
  {
    // for now assume the robot can reach on a single step any of the 8 adjacent grid points
    possible_actions = {
      {-1, -1},  // goes up & left
      {-1, 0},   // goes up
      {-1, 1},   // goes up & right
      {0, -1},   // goes left
      {0, 1},    // goes right
      {1, -1},   // goes down & left
      {1, 0},    // goes down
      {1, 1}     // goes down & right
    };
  }

  std::vector<Point> reachableLocations(
    const std::shared_ptr<OccupancyGrid> & og, const Point & point) override
  {
    std::vector<Point> points;

    for (const auto & action : possible_actions) {
      auto possible_location = getNextPoint(point, action);
      if (!og->isOutOfBounds(possible_location) && !og->isOccupied(possible_location)) {
        points.push_back(possible_location);
      }
    }

    return points;
  }

  Point getNextPoint(const Point & point, const std::vector<int> & action) const
  {
    // TODO(orduno): if Point only stores unsigned ints this could result in an invalid point.
    return Point{point.row + action[0], point.column + action[1]};
  }

private:
  std::string name;

  std::vector<std::vector<int>> possible_actions;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__FULL_MOTION_HPP_
