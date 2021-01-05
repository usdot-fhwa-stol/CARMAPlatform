/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <cmath>
#include "cost_utils.hpp"
#include "cost_safety.hpp"

namespace cost_plugin_system
{

CostofSafety::CostofSafety(double speed_limit)
{
    speed_limit_ = speed_limit;
}

double CostofSafety::compute_cost(cav_msgs::ManeuverPlan plan) const
{
    double cost = 0.0;
    int maneuver_size = sizeof(plan.maneuvers);
    for (auto it = plan.maneuvers.begin(); it != plan.maneuvers.end(); it++)
    {
        double average_speed = (cost_utils::get_maneuver_start_speed(*it) + cost_utils::get_maneuver_end_speed(*it)) / 2;
        cost += pow(average_speed, 2.0) - (1 + pow(speed_limit_, 2.0)) / pow(speed_limit_, 2.0) * average_speed + 1;
    }

    // Normalize the cost
    cost = cost / (pow(speed_limit_, 2.0)* maneuver_size);

    return cost;
}
} // namespace cost_plugin_system