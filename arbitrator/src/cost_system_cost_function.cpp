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

#include "cost_system_cost_function.hpp"
#include "arbitrator_utils.hpp"
#include "cav_srvs/ComputePlanCost.h"
#include "cav_msgs/ManeuverParameters.h"
#include <limits>

namespace arbitrator
{
    void CostSystemCostFunction::init(ros::NodeHandle &nh)
    {
        cost_system_sc_ = nh.serviceClient<cav_srvs::ComputePlanCost>("compute_plan_cost");
        initialized_ = true;
    }

    double CostSystemCostFunction::compute_total_cost(const cav_msgs::ManeuverPlan& plan)
    {
        if (!initialized_) {
            throw std::logic_error("Attempt to use CostSystemCostFunction before initialization.");
        }

        double total_cost = std::numeric_limits<double>::infinity();

        cav_srvs::ComputePlanCost service_message;
        service_message.request.maneuver_plan = plan;

        if (cost_system_sc_.call(service_message)){
            total_cost = service_message.response.plan_cost;
        } else {
            ROS_WARN_STREAM("Unable to get cost for plan from CostPluginSystem due to service call failure.");
        }

        return total_cost;
    }

    double CostSystemCostFunction::compute_cost_per_unit_distance(const cav_msgs::ManeuverPlan& plan)
    {
        double plan_dist = arbitrator_utils::get_plan_end_distance(plan) - arbitrator_utils::get_plan_start_distance(plan);
        return compute_total_cost(plan) / plan_dist;
    }
}

