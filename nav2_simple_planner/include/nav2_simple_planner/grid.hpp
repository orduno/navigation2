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

#ifndef NAV2_SIMPLE_PLANNER__GRID_HPP_
#define NAV2_SIMPLE_PLANNER__GRID_HPP_

#include <vector>
#include <ostream>

#include "nav2_simple_planner/path.hpp"

namespace nav2_simple_planner
{

template <typename T>
struct Grid
{
  Grid(const unsigned int num_rows, const unsigned int num_cols)
  {
    m.resize(num_rows);
    for (unsigned int r = 0 ; r < num_rows; r++ ) {
      m[r].resize(num_cols);
    }
  }

  Grid(const ::std::initializer_list<std::vector<T>> i_list)
    : m(i_list)
  {
  }

  void setAll(const T & value)
  {
    for (auto & row : m) {
      for (auto & cell : row) {
        cell = value;
      }
    }
  }

  void set(const Point & location, const T & value)
  {
    m[location.row][location.column] = value;
  }

  unsigned int num_rows() const { return m.size(); }

  unsigned int num_cols() const { return m[0].size(); }

  bool isEmpty() const { return m[0].size() < 1; }

  bool isWithinBounds(const Point & location) const
  {
    return !isOutOfBounds;
  }

  bool isOutOfBounds(const Point & location) const
  {
    return
      static_cast<unsigned int>(location.row) < 0 ||
      static_cast<unsigned int>(location.column) < 0 ||
      static_cast<unsigned int>(location.row) > num_rows() - 1 ||
      static_cast<unsigned int>(location.column) > num_cols() - 1;
  }

protected:

  // TODO(orduno) Implement the 1D version?
  std::vector<std::vector<T>> m;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__GRID_HPP_
