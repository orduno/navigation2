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

#ifndef NAV2_SIMPLE_PLANNER__ORDERED_GRID_HPP_
#define NAV2_SIMPLE_PLANNER__ORDERED_GRID_HPP_

#include <iomanip>

#include "nav2_simple_planner/grid.hpp"

namespace nav2_simple_planner
{

// A grid where each cell contains a non-repeating number

struct OrderedGrid : Grid<unsigned int>
{
  OrderedGrid() = delete;

  OrderedGrid(const unsigned int num_rows, const unsigned int num_cols)
  : Grid<unsigned int>(num_rows, num_cols), count(0)
  {
  }

  bool add(const Point & point)
  {
    if (m[point.row][point.column] != 0 || isOutOfBounds(point)) {
      // already added, assigning a new order is not supported yet
      return false;
    }

    m[point.row][point.column] = ++count;

    return true;
  }

  friend std::ostream & operator<<(std::ostream & os, const OrderedGrid & grid)
  {
    for (const auto & row : grid.m) {
      os << std::endl;
      for (const auto & cell : row) {
        os << std::setw(4) << cell;
      }
    }
    return os;
  }

private:
  unsigned int count;

};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__ORDERED_GRID_HPP_
