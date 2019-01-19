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

#ifndef NAV2_SIMPLE_PLANNER__CHAR_GRID_HPP_
#define NAV2_SIMPLE_PLANNER__CHAR_GRID_HPP_

#include "nav2_simple_planner/grid.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"

namespace nav2_simple_planner
{

// A grid where each cell contains a char

enum class Marker : char
{
  Empty = ' ',
  Occupied = '#',
  Path = 'o',
  Start = 's',
  Goal = 'g'
};

struct CharGrid : Grid<char>
{
  CharGrid(
    const unsigned int num_rows, const unsigned int num_cols, const Marker & marker = Marker::Empty)
  : Grid<char>(num_rows, num_cols)
  {
    setAll(static_cast<char>(marker));
  }

  void operator*=(const OccupancyGrid & occupancy)
  {
    if (occupancy.num_rows() == this->num_rows() && occupancy.num_cols() == this->num_cols()) {
      for (unsigned int row = 0; row < this->num_rows(); row++) {
        for (unsigned int col = 0; col < this->num_cols(); col++) {
          if (occupancy.isOccupied(Point{static_cast<int>(row), static_cast<int>(col)})) {
            m[row][col] = static_cast<char>(Marker::Occupied);
          }
        }
      }
    } else {
      throw std::runtime_error("Operand grids have different sizes");
    }
  }

  void set(const Point & location, const Marker & marker)
  {
    m[location.row][location.column] = static_cast<char>(marker);
  }

  friend std::ostream & operator<<(std::ostream & os, const CharGrid & grid)
  {
    for (const auto & row : grid.m) {
      os << std::endl;
      for (const auto & cell : row) {
        os << " " << cell << " ";
      }
    }
    return os;
  }

};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__CHAR_GRID_HPP_
