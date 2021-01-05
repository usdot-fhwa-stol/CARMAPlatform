/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#ifndef __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_
#define __ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_

#include "capabilities_interface.hpp"
#include "plugin_neighbor_generator.hpp"
#include <cav_srvs/PlanManeuvers.h>
#include <map>

namespace arbitrator
{
    template <class T>
    std::vector<cav_msgs::ManeuverPlan> PluginNeighborGenerator<T>::generate_neighbors(cav_msgs::ManeuverPlan plan) const
    {
        cav_srvs::PlanManeuvers msg;
        msg.request.prior_plan = plan;
        std::map<std::string, cav_srvs::PlanManeuvers> res = ci_.multiplex_service_call_for_capability(CapabilitiesInterface::STRATEGIC_PLAN_CAPABILITY, msg);

        // Convert map to vector of map values
        std::vector<cav_msgs::ManeuverPlan> out;
        for (auto it = res.begin(); it != res.end(); it++)
        {
            out.push_back(it->second.response.new_plan);
        }
        return out;
    }
}

#endif //__ARBITRATOR_INCLUDE_PLUGIN_NEIGHBOR_GENERATOR_TPP__ARBITRATOR_INCLUDE_
