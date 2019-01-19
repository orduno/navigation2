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

#ifndef NAV2_SIMPLE_PLANNER__COST_GRID_HPP_
#define NAV2_SIMPLE_PLANNER__COST_GRID_HPP_

#include "nav2_simple_planner/grid.hpp"

namespace nav2_simple_planner
{

// grid format:
// 255 = no information
// 254 = definitely in collision
// 253 to 128 = possibly in collision
// 127 = definitely not in collision

typedef uint8_t CostValue;

struct CostGrid : Grid<CostValue>
{
  static const CostValue no_information;
  static const CostValue lethal_obstacle;
  static const CostValue inscribed_inflated_obstacle;
  static const CostValue circumscribed_inflated_obstacle;
  static const CostValue free_space;

  bool isOccupied(const Point & point) const
  {
    return m[point.row][point.column] >= circumscribed_inflated_obstacle;
  }

  friend std::ostream & operator<<(std::ostream & os, const Grid & grid)
  {
    for (const auto & row : grid.m) {
      os << std::endl;
      for (const auto & cell_value : row) {
        os << (isOccupied(cell_value) ? "# " : ". ");
      }
    }
    return os;
  }
};

const CostValue CostGrid::no_information = 255;
const CostValue CostGrid::lethal_obstacle = 254;
const CostValue CostGrid::inscribed_inflated_obstacle = 253;
const CostValue CostGrid::circumscribed_inflated_obstacle = 128;
const CostValue CostGrid::free_space = 0;

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__COST_GRID_HPP_
