/*
 * Copyright (C) 2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "capabilities_interface.hpp"
#include "plugin_neighbor_generator.hpp"
#include <cav_srvs/PlanManeuvers.h>
#include <map>

namespace arbitrator
{
    void PluginNeighborGenerator::initalize() 
    {

    }

    std::vector<cav_msgs::ManeuverPlan> PluginNeighborGenerator::generate_neighbors(cav_msgs::ManeuverPlan plan) const
    {
        cav_srvs::PlanManeuversRequest req;
        req.prior_plan = plan;
        std::map<std::string, cav_srvs::PlanManeuversResponse> res = ci_.
            multiplex_service_call_for_capability<cav_srvs::PlanManeuversRequest, 
            cav_srvs::PlanManeuversResponse>("PLAN_STRATEGIC", req);

        // Convert map to vector of map values
        std::vector<cav_msgs::ManeuverPlan> out;
        for (auto it = res.begin(); it != res.end(); it++)
        {
            out.push_back(it->second.new_plan);
        }

        return out;
    }
}