// Copyright (c) 2020, Samsung Research America
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

#ifndef SMAC_PLANNER__NODE_BASIC_HPP_
#define SMAC_PLANNER__NODE_BASIC_HPP_

#include <math.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "ompl/base/StateSpace.h"
#include "smac_planner/constants.hpp"
#include "smac_planner/types.hpp"

namespace smac_planner
{
/**
 * @class smac_planner::NodeBasic
 * @brief NodeBasic implementation for priority queue insertion
 */
template<typename NodeT>
class NodeBasic
{
public:
  /**
   * @brief A constructor for smac_planner::NodeBasic
   * @param cost_in The costmap cost at this node
   * @param index The index of this node for self-reference
   */
  explicit NodeBasic(const unsigned int index)
  : index(index), graph_node_ptr(nullptr) {}

  typename NodeT::Coordinates pose;  // Used by NodeSE2
  NodeT * graph_node_ptr;
  unsigned int index;
};

}  // namespace smac_planner

#endif  // SMAC_PLANNER__NODE_BASIC_HPP_
