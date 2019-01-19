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

#ifndef NAV2_SIMPLE_PLANNER__PLANNER_HPP_
#define NAV2_SIMPLE_PLANNER__PLANNER_HPP_

#include <memory>
#include <functional>

#include "nav2_simple_planner/motion.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"
#include "nav2_simple_planner/point.hpp"
#include "nav2_simple_planner/connected_grid.hpp"

namespace nav2_simple_planner
{

class Planner
{
public:
  Planner(const std::shared_ptr<Motion> & motion, const std::shared_ptr<Logger> & logger);

  Planner() = delete;

  void enableRuntimeLogging() { logger_->enable(); }

  void disableRuntimeLogging() { logger_->disable(); }

  // Provide parameters to the search problem
  // returns true if the problem is well-posed (goal or start are not invalid), false otherwise
  bool setSearchProblem(
    std::shared_ptr<OccupancyGrid> occupancy, const Point & goal, const Point & start);

  void setHeuristic(const std::function<double (const Point &, const Point &)> & heuristic)
  {
    heuristic_ = heuristic;
  }

  // Search for the goal. In the process, calculate for each explored cell, from which one is coming.
  bool searchForGoal(ConnectedGrid & connections);

  // If the goal is found, retrieve the path from a connection graph
  Path getPath(const ConnectedGrid & connections);

private:
  std::shared_ptr<Motion> motion_;
  std::shared_ptr<Logger> logger_;

  // Search input variables
  Point goal_;
  Point start_;
  std::shared_ptr<OccupancyGrid> occupancy_;

  // The path found
  Path path_;

  // Search parameters
  double step_cost_;
  std::function<double(const Point &, const Point &)> heuristic_;

  bool areInvalidArguments(
    const OccupancyGrid & occupancy, const Point & goal, const Point & start) const;

  void reset();

  // g_value represents the number of steps taken from starting point
  std::vector<double> getCellValues(const Point & p, const double g_value) const;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__PLANNER_HPP_
