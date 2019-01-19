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

#ifndef NAV2_SIMPLE_PLANNER__CONNECTED_GRID_HPP_
#define NAV2_SIMPLE_PLANNER__CONNECTED_GRID_HPP_

#include <iomanip>
#include <memory>

#include "nav2_simple_planner/grid.hpp"
#include "nav2_simple_planner/point.hpp"

namespace nav2_simple_planner
{

struct Connection
{
  Point from;
  bool isConnected = false;
};

// A grid where each cell contains a connection

struct ConnectedGrid : Grid<Connection>
{
  ConnectedGrid() = delete;

  ConnectedGrid(const unsigned int num_rows, const unsigned int num_cols)
  : Grid<Connection>(num_rows, num_cols)
  {
  }

  bool add(const Point & to, const Point & from)
  {
    if (isOutOfBounds(to)) {
      return false;
    }

    m[to.row][to.column].from = from;
    m[to.row][to.column].isConnected = true;

    return true;
  }

  Connection getConnection(const Point & point) const
  {
    return m[point.row][point.column];
  }

  friend std::ostream & operator<<(std::ostream & os, const ConnectedGrid & grid)
  {
    for (const auto & row : grid.m) {
      os << std::endl;
      for (const auto & cell : row) {
        if (cell.isConnected) {
          os << std::setw(2) << cell.from.row << "," << cell.from. column;
        } else {
          os << std::setw(4) << "---";
        }
      }
    }
    return os;
  }
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__CONNECTED_GRID_HPP_
