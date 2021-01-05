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

#ifndef __ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__
#define __ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__

#include <vector>
#include <cav_msgs/ManeuverPlan.h>

namespace arbitrator
{
    /**
     * Generic interface representing computation of children or neighbor nodes 
     * in a planning search graph
     */
    class NeighborGenerator
    {
        public:
            /**
             * \brief Generate the list of neighbors/children that a given node in the search graph
             *      expands to
             * \param plan The maneuver plan to expand upon
             * \return A vector containing the new plans generated from it, if any
             */
            virtual std::vector<cav_msgs::ManeuverPlan> generate_neighbors(cav_msgs::ManeuverPlan plan) const = 0;

            /**
             * \brief Virtual destructor provided for memory safety
             */
            virtual ~NeighborGenerator(){};
    };
}

#endif //__ARBITRATOR_INCLUDE_NEIGHBOR_GENERATOR_HPP__