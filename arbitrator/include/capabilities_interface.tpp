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

#ifndef __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_TPP__ARBITRATOR_INCLUDE_
#define __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_TPP__ARBITRATOR_INCLUDE_

#include <vector>
#include <map>
#include <string>
#include <functional>
#include <cav_srvs/PlanManeuvers.h>

namespace arbitrator 
{
    template<typename MSrv>
    std::map<std::string, MSrv> CapabilitiesInterface::multiplex_service_call_for_capability(std::string query_string, MSrv msg)
    {
        std::vector<std::string> topics = get_topics_for_capability(query_string);
        std::map<std::string, MSrv> responses;
        for (auto i = topics.begin(); i != topics.end(); i++) 
        {
            ros::ServiceClient sc = nh_->serviceClient<cav_srvs::PlanManeuvers>(*i);
            if (sc.call(msg)) {
                responses.emplace(*i, msg);
            }
        }
        return responses;
    }
};

#endif
