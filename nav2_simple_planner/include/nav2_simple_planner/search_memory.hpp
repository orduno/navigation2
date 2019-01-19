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

#ifndef NAV2_SIMPLE_PLANNER__SEARCH_MEMORY_HPP_
#define NAV2_SIMPLE_PLANNER__SEARCH_MEMORY_HPP_

#include <memory>

#include "nav2_simple_planner/point.hpp"
#include "nav2_simple_planner/cell.hpp"
#include "nav2_simple_planner/cell_set.hpp"
#include "nav2_simple_planner/point_set.hpp"
#include "nav2_simple_planner/ordered_grid.hpp"

namespace nav2_simple_planner
{

// Group of data structures used during the search process

template <typename T>
class SearchMemory
{
public:
  SearchMemory(const unsigned int num_rows, const unsigned int num_cols)
  : current_cell_(Cell<T>{std::vector<T>{}, Point{0, 0}}),
    expansion_order_(std::make_shared<OrderedGrid>(OrderedGrid{num_rows, num_cols}))
  {
  }

  bool addUnexpanded(const Point & p, const std::vector<T> & values)
  {
    return unexplored_cells_.add(Cell<T>{values, p});
  }

  bool addVisited(const Point & p)
  {
    return explored_points_.add(p);
  }

  bool wasVisited(const Point & p)
  {
    return explored_points_.contains(p);
  }

  bool updateCurrent()
  {
    Cell<T> cell;
    if (unexplored_cells_.get_next_cell(cell)) {
      current_cell_ = cell;
      expansion_order_->add(cell.location);
      return true;
    }
    return false;
  }

  Cell<T> current()
  {
    return current_cell_;
  }

  std::shared_ptr<OrderedGrid> expansionOrder()
  {
    return expansion_order_;
  }

private:
  Cell<T> current_cell_;

  CellSet<T> unexplored_cells_;

  PointSet explored_points_;

  std::shared_ptr<OrderedGrid> expansion_order_;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__SEARCH_MEMORY_HPP_
