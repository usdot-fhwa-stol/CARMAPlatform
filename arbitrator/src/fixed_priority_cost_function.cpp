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

#include "fixed_priority_cost_function.hpp"
#include "arbitrator_utils.hpp"
#include "cav_msgs/ManeuverParameters.h"
#include <limits>

namespace arbitrator
{
    FixedPriorityCostFunction::FixedPriorityCostFunction(const std::map<std::string, double> &plugin_priorities) 
    {
        // Identify the highest priority values present in the list
        double max_priority = std::numeric_limits<double>::lowest();
        for (auto it = plugin_priorities.begin(); it != plugin_priorities.end(); it++)
        {
            if (it->second > max_priority)
            {
                max_priority = it->second;
            }
        }

        // Normalize the list and invert into costs
        for (auto it = plugin_priorities.begin(); it != plugin_priorities.end(); it++)
        {
            plugin_costs_[it->first] = 1.0 - (it->second / max_priority);
        }
    }

    double FixedPriorityCostFunction::compute_total_cost(const cav_msgs::ManeuverPlan& plan) 
    {
        double total_cost = 0.0;
        for (auto it = plan.maneuvers.begin(); it != plan.maneuvers.end(); it++)
        {
            std::string planning_plugin = GET_MANEUVER_PROPERTY(*it, parameters).planning_strategic_plugin;
            total_cost += (arbitrator_utils::get_maneuver_end_distance(*it) - arbitrator_utils::get_maneuver_start_distance(*it)) *
                plugin_costs_.at(planning_plugin);
        }

        return total_cost;
    }

    double FixedPriorityCostFunction::compute_cost_per_unit_distance(const cav_msgs::ManeuverPlan& plan)
    {
        double plan_dist = arbitrator_utils::get_plan_end_distance(plan) - arbitrator_utils::get_plan_start_distance(plan);
        return compute_total_cost(plan) / plan_dist;
    }
}
