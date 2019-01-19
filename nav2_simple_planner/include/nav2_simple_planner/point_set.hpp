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

#ifndef NAV2_SIMPLE_PLANNER__POINT_SET_HPP_
#define NAV2_SIMPLE_PLANNER__POINT_SET_HPP_

#include <unordered_set>

#include "nav2_simple_planner/point.hpp"

namespace nav2_simple_planner
{

class PointSet {
public:
  bool add(const Point & point)
  {
    return list.insert(point).second;
  }

  bool contains(const Point & point) const
  {
    return list.find(point) != list.end();
  }

private:
  std::unordered_set<Point> list;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__POINT_SET_HPP_
