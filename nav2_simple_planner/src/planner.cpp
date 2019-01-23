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

#include <memory>
#include <functional>

#include "nav2_simple_planner/planner.hpp"
#include "nav2_simple_planner/motion.hpp"
#include "nav2_simple_planner/point.hpp"
#include "nav2_simple_planner/connected_grid.hpp"
#include "nav2_simple_planner/char_grid.hpp"
#include "nav2_simple_planner/search_memory.hpp"

namespace nav2_simple_planner
{

Planner::Planner(
  const std::shared_ptr<World> world,
  const std::shared_ptr<Motion> & motion,
  const std::shared_ptr<Logger> & logger)
: world_(world),
  motion_(motion),
  logger_(logger),
  step_cost_(0.5)
{
}

bool Planner::setSearchProblem(const Point &goal, const Point &start)
{
  if (!areValidArguments(goal, start))
  {
    reset();
    return false;
  }

  goal_ = goal;
  start_ = start;

  return true;
}

bool Planner::searchForGoal(ConnectedGrid & connections)
{
  SearchMemory<double> memory(world_->numCellsY(), world_->numCellsX());
  memory.addUnexpanded(start_, getCellValues(start_, 0));
  memory.addVisited(start_);

  // memory.addVisitedandUnexpanded(start_, getCellValues(start_, 0));

  bool found_path = false;
  bool resign = false;

  // while (memory.hasMoreToExplore() && memory.current().location != goal) {
  // // keep looking
  //   continueExpanding(memory);
  // }
  while (!found_path && !resign)
  {
    // Keep looking
    if (memory.updateCurrent())
    {
      // Got a cell that hasn't been expanded.
      logger_->debug() << "Exploring location " << memory.current();
      if (memory.current().location == goal_)
      {
        // This cell is the goal!
        found_path = true;
        logger_->info() << "Found a path to the goal!";
      }
      else
      {
        // Add cells reachable from this cell.
        auto reachable = motion_->reachableLocations(world_, memory.current().location);
        for (const auto & p : reachable)
        {
          // Go over each point
          if (!memory.wasVisited(p))
          {
            // Location hasn't been expanded, let's calculate costs and add it
            memory.addUnexpanded(p, getCellValues(p,
              // cost of parent cell + cost of taking stepping + cost of child cell
              // TODO(orduno) scale the cell cost correctly
              memory.current().values[1] + step_cost_ + 0.05 * world_->getCost(
                static_cast<unsigned int>(p.column), static_cast<unsigned int>(p.row))));
            memory.addVisited(p);
            connections.add(p, memory.current().location);
          }
        }
      }
    }
    else
    {
      // Nothing else to explore
      resign = true;
      logger_->warn() << "Resigning, no more cells to explore";
    }
  }

  // logger_->debug() << "Expansion Order: " << *memory.expansionOrder();
  // logger_->debug() << "Connections: " << connections << std::endl;

  return found_path;
}

Path Planner::getPath(const ConnectedGrid & connections)
{
  Path path;

  // Traverse the connections to retrieve the path
  Connection connection;
  connection.from = goal_;
  connection.isConnected = true;

  // Grid for visualization
  CharGrid viz_grid(world_->numCellsY(), world_->numCellsX());

  while (connection.isConnected)
  {
    path.add(connection.from);
    viz_grid.set(connection.from, Marker::Path);
    connection = connections.getConnection(connection.from);
  }

  // TODO(orduno) broken for now given the recent refactor
  // Mask with the occupancy gid to add obstacles
  // viz_grid *= *occupancy_;
  // viz_grid.set(start_, Marker::Start);
  // viz_grid.set(goal_, Marker::Goal);

  // logger_->debug() << "Path Graph: " << viz_grid;

  return path;
}

bool Planner::areValidArguments(const Point &goal, const Point &start) const
{
  if (world_->isEmpty())
  {
    logger_->error("The provided world is empty");
    return false;
  }

  if (!world_->isWithinBounds(
    static_cast<unsigned int>(goal.column),
    static_cast<unsigned int>(goal.row)))
  {
    logger_->error("The provided goal is outside of world");
    return false;
  }

  if (!world_->isWithinBounds(
    static_cast<unsigned int>(start.column),
    static_cast<unsigned int>(start.row)))
  {
    logger_->error("The provided start is outside of world");
    return false;
  }

  return true;
}

void Planner::reset()
{
  goal_ = Point{};
  start_ = Point{};
  path_ = Path{};
}

std::vector<double> Planner::getCellValues(const Point & p, const double g_value) const
{
  double h_value = heuristic_(p, goal_);
  double f_value = g_value + h_value;
  return {f_value, g_value, h_value};
}

}  // namespace nav2_simple_planner
