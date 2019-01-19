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

#ifndef NAV2_SIMPLE_PLANNER__OCCUPANCY_GRID_HPP_
#define NAV2_SIMPLE_PLANNER__OCCUPANCY_GRID_HPP_

#include "nav2_simple_planner/grid.hpp"

namespace nav2_simple_planner
{

// grid format:
//  false = navigable space
//  true = occupied space

struct OccupancyGrid : Grid<bool>
{
  OccupancyGrid(const ::std::initializer_list<std::vector<bool>> i_list)
    : Grid<bool>(i_list)
  {
  }

  bool isOccupied(const Point & point) const
  {
    return m[point.row][point.column];
  }

  friend std::ostream & operator<<(std::ostream & os, const OccupancyGrid & grid)
  {
    for (const auto & row : grid.m) {
      os << std::endl;
      for (const auto & cell : row) {
        os << (cell ? "# " : ". ");
      }
    }
    return os;
  }

};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__OCCUPANCY_GRID_HPP_
