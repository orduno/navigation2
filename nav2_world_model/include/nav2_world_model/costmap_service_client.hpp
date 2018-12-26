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

#ifndef NAV2_WORLD_MODEL__COSTMAP_SERVICE_CLIENT_HPP_
#define NAV2_WORLD_MODEL__COSTMAP_SERVICE_CLIENT_HPP_

#include "nav2_util/service_client.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"

namespace nav2_world_model
{

using nav2_tasks::ServiceClient;
using nav2_msgs::srv::GetCostmap;
using CostmapServiceRequest =
  ServiceClient<GetCostmap>::RequestType;
using CostmapServiceResponse =
  ServiceClient<GetCostmap>::ResponseType;

class CostmapServiceClient : public ServiceClient<GetCostmap>
{
public:
  CostmapServiceClient()
  : ServiceClient<GetCostmap>("GetCostmap")
  {
  }
};

}  // namespace nav2_world_model

#endif  // NAV2_WORLD_MODEL__COSTMAP_SERVICE_CLIENT_HPP_
