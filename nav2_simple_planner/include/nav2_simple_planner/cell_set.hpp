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

#ifndef NAV2_SIMPLE_PLANNER__CELL_SET_HPP_
#define NAV2_SIMPLE_PLANNER__CELL_SET_HPP_

#include <set>

#include "nav2_simple_planner/cell.hpp"

namespace nav2_simple_planner
{

template <typename T>
class CellSet {
public:
  bool add(const Cell<T> & cell)
  {
    auto result = set.insert(cell);
    return result.second;
  }

  bool get_next_cell(Cell<T> & cell)
  {
    auto it_first = set.begin();

    if (it_first != set.end()) {
      cell = *it_first;
      set.erase(it_first);

      return true;
    }

    return false;
  }

  bool is_empty()
  {
    return set.empty();
  }

private:
  // TODO(orduno) Replace for priority queue
  //              https://en.cppreference.com/w/cpp/container/priority_queue
  std::set<Cell<T>, CellComparator<T>> set;
};

}  // namespace nav2_simple_planner

#endif  // NAV2_SIMPLE_PLANNER__CELL_SET_HPP_
