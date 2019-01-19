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

#ifndef NAV2_SIMPLE_PLANNER__POINT_HPP_
#define NAV2_SIMPLE_PLANNER__POINT_HPP_

#include <ostream>

namespace nav2_simple_planner
{

struct Point
{
  int row, column;

  Point()
  : row(0), column(0) { }

  // Point(const unsigned int row, const unsigned int column)
  Point(const int row, const int column)
  : row(row), column(column) { }

  bool operator ==(const Point & obj) const
  {
    if (row == obj.row && column == obj.column) {
      return true;
    } else {
      return false;
    }
  }

  friend std::ostream & operator<<(std::ostream & os, const Point & point)
  {
    return os
      << "row: " << point.row
      << " column: " << point.column;
  }
};

}  // namespace nav2_simple_planner

namespace std
{
  template <>
  struct hash<nav2_simple_planner::Point>
  {
      size_t operator()(nav2_simple_planner::Point const & obj) const noexcept
      {
        return (
          (51 + std::hash<int>()(obj.row)) * 51
          + std::hash<int>()(obj.column)
        );
      }
  };
}

#endif  // NAV2_SIMPLE_PLANNER__POINT_HPP_
