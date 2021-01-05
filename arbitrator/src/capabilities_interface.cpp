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

#include "capabilities_interface.hpp"
#include <cav_srvs/PlanManeuvers.h>
#include <exception>

namespace arbitrator
{
    const std::string CapabilitiesInterface::STRATEGIC_PLAN_CAPABILITY = "strategic_plan/plan_maneuvers";
    
    std::vector<std::string> CapabilitiesInterface::get_topics_for_capability(const std::string& query_string)
    {
        std::vector<std::string> topics = {};

        cav_srvs::GetPluginApi srv;
        srv.request.capability = "";

        if (query_string == STRATEGIC_PLAN_CAPABILITY && sc_s.call(srv))
        {
            topics = srv.response.plan_service;
            ROS_INFO_STREAM("Received Topic: " << topics.front());
        }

        return topics;

    }
}
