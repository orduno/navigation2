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
// limitations under the License. Reserved.

#include <gtest/gtest.h>
#include <iostream>
#include <memory>

#include "nav2_simple_planner/occupancy_grid.h"
#include "nav2_simple_planner/astar_planner.h"
#include "nav2_simple_planner/robot.h"
#include "nav2_simple_planner/logger.h"

using namespace std;
using namespace nav2_simple_planner;

int main()
{
  OccupancyGrid open {
  // 0  1  2  3  4  5  6  7  8  9
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 0
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 1
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 5
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 7
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 8
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 9
  };

  OccupancyGrid maze1 {
  // 0  1  2  3  4  5  6  7  8  9
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // 0
    {0, 0, 1, 0, 1, 0, 1, 0, 1, 0}, // 1
    {1, 0, 1, 0, 1, 0, 1, 0, 0, 1}, // 2
    {0, 0, 1, 0, 1, 0, 1, 1, 0, 0}, // 3
    {0, 0, 1, 0, 1, 0, 0, 1, 0, 0}, // 4
    {0, 0, 1, 0, 1, 1, 0, 0, 1, 0}, // 5
    {0, 0, 0, 0, 0, 0, 1, 0, 1, 0}, // 6
    {0, 0, 1, 0, 1, 0, 0, 0, 1, 0}, // 7
    {0, 0, 1, 0, 1, 1, 1, 1, 1, 0}, // 8
    {0, 0, 1, 0, 0, 0, 0, 0, 1, 0}, // 9
  };

  OccupancyGrid maze2 {
  // 0  1  2  3  4  5  6  7  8  9
    {0, 0, 1, 0, 0, 0, 0, 0, 0, 0}, // 0
    {0, 0, 0, 0, 1, 0, 1, 0, 1, 0}, // 1
    {1, 0, 1, 0, 1, 0, 1, 0, 0, 1}, // 2
    {0, 0, 1, 0, 0, 0, 1, 1, 0, 0}, // 3
    {0, 0, 1, 0, 1, 0, 0, 1, 0, 0}, // 4
    {0, 0, 1, 0, 1, 1, 0, 0, 1, 0}, // 5
    {0, 0, 0, 0, 0, 0, 1, 0, 0, 0}, // 6
    {0, 0, 1, 0, 1, 0, 0, 0, 1, 0}, // 7
    {0, 0, 1, 0, 1, 1, 1, 1, 1, 0}, // 8
    {0, 0, 1, 0, 0, 0, 0, 0, 1, 0}, // 9
  };

  OccupancyGrid maze3 {
  // 0  1  2  3  4  5  6  7  8  9
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 0
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 1
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 2
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 4
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 5
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 6
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 7
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // 8
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 9
  };

  OccupancyGrid simple {
  // 0  1  2  3
    {0, 1, 0, 0}, // 0
    {0, 0, 0, 0}, // 1
    {0, 1, 1, 0}, // 2
    {0, 0, 0, 0}, // 3
  };

  AStarPlanner planner();

  auto map = make_shared<OccupancyGrid>(maze1);

  if (!planner.computePath(map, Point{9, 9}, Point{0, 0})) {
    logger->error("Planner failed.");
  } else {
    logger->info() << "Occupancy: " << *map;
    logger->info() << "Path: " << planner.getPath();
  }

  getchar();
  return 0;
}
