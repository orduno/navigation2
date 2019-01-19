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

#ifndef NAV2_SIMPLE_PLANNER__PATH_HPP_
#define NAV2_SIMPLE_PLANNER__PATH_HPP_

#include <vector>
#include <ostream>

#include "nav2_simple_planner/point.hpp"

namespace nav2_simple_planner
{

struct Path
{
  unsigned int size() const { return v.size(); }

  Point at(const unsigned int n) { return v.at(n); }

  void add(Point p) { v.push_back(p); }

  friend std::ostream & operator<<(std::ostream & os, const Path & path)
  {
    for (const Point & point : path.v) {
      os << point.row << "," << point.column << "  ";
    }
    return os;
  }

  std::vector<Point> getPoints() const { return v; }

private:
  std::vector<Point> v;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__PATH_HPP_
