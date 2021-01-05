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
#pragma once

#include <cav_msgs/ManeuverPlan.h>

namespace cost_plugin_system
{
class CostPlugins
{

public:
    /**
     * \brief Compute the cost of a given maneuver plan
     * \param plan The plan to evaluate
     * \return double The total cost
     */
    virtual double compute_cost(cav_msgs::ManeuverPlan plan) const = 0;

    /**
     * \brief Virtual destructor provided for memory safety
     */
    virtual ~CostPlugins(){};
};
} // namespace cost_plugin_system
