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

#ifndef NAV2_SIMPLE_PLANNER__CELL_HPP_
#define NAV2_SIMPLE_PLANNER__CELL_HPP_

#include <vector>
#include <memory>

#include "nav2_simple_planner/point.hpp"

namespace nav2_simple_planner
{

template <typename T>
struct Cell
{
  Cell()
  : location(Point()),
    values(std::vector<T>{})
  {
  }

  Cell(const Cell & other)
  : location(other.location),
    values(other.values)
  {
  }

  Cell(const std::vector<T> & values, const Point & location)
  : location(location),
    values(values)
  {
  }

  Cell & operator=(const Cell & other)
  {
    if (&other == this) {
      return *this;
    }

    values = other.values;
    location = other.location;
    return *this;
  }

  bool operator<(const Cell<T> & rhs) const
  {
    return values[0] < rhs.values[0];
  }

  friend std::ostream & operator<<(std::ostream & os, const Cell & other)
  {
    os << "at " << other.location;

    os << " with values [ ";
    for (const auto & value : other.values){
      os << value << " ";
    }
    os << "]";

    return os;
  }

  // A cell has a location defined by a point
  Point location;

  // A cell can contain multiple values (of same type)
  std::vector<T> values;
};

// Comparator used for defining the sorting preference in CellSet
// first sort by the first element in values, then by row, then by column
template <typename T>
struct CellComparator
{
  bool operator() (const Cell<T> a, const Cell<T> b) const
  {
    if (a.values[0] == b.values[0]) {
      if (a.location.row == b.location.row) {
        if (a.location.column == b.location.column) {
          return false;
        } else {
          return a.location.column < b.location.column;
        }
      } else {
        return a.location.row < b.location.row;
      }
    }
    return a.values[0] < b.values[0];
  }
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__CELL_HPP_
