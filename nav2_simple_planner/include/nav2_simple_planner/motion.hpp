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

#ifndef NAV2_SIMPLE_PLANNER__MOTION_HPP_
#define NAV2_SIMPLE_PLANNER__MOTION_HPP_

#include <vector>
#include <string>
#include <memory>

#include "nav2_simple_planner/point.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"
#include "nav2_simple_planner/logger.hpp"

namespace nav2_simple_planner
{

/*
 * An abstract class for defining location translation
 */

class Motion
{
public:
  virtual ~Motion() = default;

  virtual std::vector<Point> reachableLocations(
    const std::shared_ptr<OccupancyGrid> & og, const Point & point) = 0;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__MOTION_HPP_
