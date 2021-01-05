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

#ifndef __ARBITRATOR_INCLUDE_FIXED_PRIORITY_COST_FUNCTION_HPP__
#define __ARBITRATOR_INCLUDE_FIXED_PRIORITY_COST_FUNCTION_HPP__

#include "cost_function.hpp"
#include <map>
#include <string>

namespace arbitrator
{
    /**
     * \brief Implementation of the CostFunction interface
     * 
     * Implements costs by associating a fixed priority number with each plugin
     * (as specified by configuration). This priority is then normalized across
     * all plugins, and then an inverse is computed to arrive at the cost per 
     * unit distance for that plugins.
     * 
     * e.g. Three plugins with priority 20, 10, and 5 will respectively have
     * costs 0, 0.5, 0.75 per unit distance.
     */
    class FixedPriorityCostFunction : public CostFunction
    {
        public:
            /**
             * \brief Constructor for FixedPriorityCostFunction
             * \param nh A publically namespaced ("/") ros::NodeHandle
             */
            FixedPriorityCostFunction(const std::map<std::string, double> &plugin_priorities);

            /**
             * \brief Compute the unit cost over distance of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost divided by the total distance of the plan
             */
            double compute_total_cost(const cav_msgs::ManeuverPlan& plan);

            /**
             * \brief Compute the unit cost over distance of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost divided by the total distance of the plan
             */
            double compute_cost_per_unit_distance(const cav_msgs::ManeuverPlan& plan);
        private:
            std::map<std::string, double> plugin_costs_;
    };
};

#endif //__ARBITRATOR_INCLUDE_FIXED_PRIORITY_COST_FUNCTION_HPP__
