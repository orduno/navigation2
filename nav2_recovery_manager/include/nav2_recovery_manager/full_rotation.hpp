// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
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

#ifndef NAV2_RECOVERY_MANAGER__FULL_ROTATION_HPP_
#define NAV2_RECOVERY_MANAGER__FULL_ROTATION_HPP_

#include <string>

#include "nav2_recovery_manager/recovery_behavior.hpp"

namespace nav2_recovery_manager
{

// Provides an interface for defining a recovery behavior
class FullRotation : public RecoveryBehavior
{
public:
  explicit FullRotation(const std::string name)
  : RecoveryBehavior(name)
  {
  }

  void initialize() override;

  bool run() override;

  std::string name() { return name_; };

protected:
  std::string name_;
};

}  // nav2_recovery_manager

#endif  // NAV2_RECOVERY_MANAGER__FULL_ROTATION_HPP_