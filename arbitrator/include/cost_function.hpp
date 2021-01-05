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

#ifndef __ARBITRATOR_INCLUDE_COST_FUNCTION_HPP__
#define __ARBITRATOR_INCLUDE_COST_FUNCTION_HPP__

#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/ManeuverPlan.h>

namespace arbitrator
{
    /**
     * \brief Generic interface representing a means of computing cost for plans
     *      in the search graph
     */
    class CostFunction
    {
        public:
            /**
             * \brief Compute the cost of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost
             */
            virtual double compute_total_cost(const cav_msgs::ManeuverPlan& plan) = 0;

            /**
             * \brief Compute the unit cost over distance of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost divided by the total distance of the plan
             */
            virtual double compute_cost_per_unit_distance(const cav_msgs::ManeuverPlan& plan) = 0;

            /**
             * \brief Virtual destructor provided for memory safety
             */
            virtual ~CostFunction(){};
    };
};

#endif //__ARBITRATOR_INCLUDE_COST_FUNCTION_HPP__