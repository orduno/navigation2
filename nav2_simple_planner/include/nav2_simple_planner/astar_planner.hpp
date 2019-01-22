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

#ifndef NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_HPP_
#define NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>

#include "nav2_simple_planner/world.hpp"
#include "nav2_simple_planner/motion.hpp"
#include "nav2_simple_planner/planner.hpp"
#include "nav2_simple_planner/path.hpp"
#include "nav2_simple_planner/logger.hpp"
#include "nav2_simple_planner/occupancy_grid.hpp"

namespace nav2_simple_planner
{

class AStarPlanner
{
public:
  explicit AStarPlanner(const std::shared_ptr<World> & world);
  AStarPlanner() = delete;

  ~AStarPlanner();

  bool computePath(const Point & goal, const Point & start);

  Path getPath() { return path_; }

  void enableLogging();
  void disableLogging();

private:
  std::shared_ptr<World> world_;
  std::shared_ptr<Motion> motion_;
  std::shared_ptr<Logger> logger_;

  Planner planner_;
  Path path_;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__ASTAR_PLANNER_HPP_
