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

#include <cmath>
#include <memory>

#include "nav2_simple_planner/astar_planner.hpp"
#include "nav2_simple_planner/full_motion.hpp"
#include "nav2_simple_planner/logger.hpp"

namespace nav2_simple_planner
{

AStarPlanner::AStarPlanner(const std::shared_ptr<World> & world)
: world_(world),
  motion_(std::make_shared<FullMotion>()),
  logger_(std::make_shared<ConsoleLogger>(LogLevel::Debug)),
  planner_(Planner{world_, motion_, logger_})
{
}

AStarPlanner::~AStarPlanner()
{
}

bool AStarPlanner::computePath(const Point & goal, const Point & start)
{
  if (!planner_.setSearchProblem(goal, start)) {
    return false;
  }

  planner_.setHeuristic( [](const Point & p1, const Point & p2) -> double {
    // Must define an optimistic heuristic for A* algorithm to work.
    // Let's use the Euclidean distance between the points.
    return std::sqrt(
      std::pow(static_cast<double>(p1.row - p2.row), 2.0)+
      std::pow(static_cast<double>(p1.column - p2.column), 2.0));
  });

  ConnectedGrid connections(world_->numCellsY(), world_->numCellsX());

  if(!planner_.searchForGoal(connections)) {
    return false;
  }

  path_ = planner_.getPath(connections);

  return true;
}

void AStarPlanner::enableLogging()
{
  planner_.enableRuntimeLogging();
}

void AStarPlanner::disableLogging()
{
  planner_.disableRuntimeLogging();
}

}  // namespace nav2_simple_planner
