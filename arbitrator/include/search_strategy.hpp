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

#ifndef __ARBITRATOR_INCLUDE_SEARCH_STRATEGY_HPP__
#define __ARBITRATOR_INCLUDE_SEARCH_STRATEGY_HPP__

#include <map>
#include <cav_msgs/ManeuverPlan.h>

namespace arbitrator
{
    /**
     * \brief Generic interface representing a computation to prioritize nodes
     *      for expansion in a search graph.
     * 
     * This interface takes in plans and their computed costs (C) and then may
     * apply a heuristic cost (H) if so desired before sorting by C+H or otherwise
     * prioritizing the potential plans.
     * 
     * It is not safe to assume that the size of the input list equals the size 
     * the list returned.
     */
    class SearchStrategy
    {
        public:
            /**
             * \brief Sort the list of plans in the open-set by priority
             * \param plans The list of (plan, cost) pairs to sort
             * \return A sorted (and/or reduced) list of (plan, cost) pairs after a
             *      heuristic may or may not have been applied
             */
            virtual std::vector<std::pair<cav_msgs::ManeuverPlan, double>> prioritize_plans(std::vector<std::pair<cav_msgs::ManeuverPlan, double>> plans) const = 0;

            /**
             * \brief Virtual destructor provided for memory safety
             */
            virtual ~SearchStrategy(){};
    }; 
} // namespace arbitrator


#endif //__ARBITRATOR_INCLUDE_SEARCH_STRATEGY_HPP__