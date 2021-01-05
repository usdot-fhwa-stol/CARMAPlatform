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

#ifndef __ARBITRATOR_INCLUDE_TREE_PLANNER_HPP__
#define __ARBITRATOR_INCLUDE_TREE_PLANNER_HPP__

#include <memory>
#include <cav_msgs/ManeuverPlan.h>
#include "planning_strategy.hpp"
#include "cost_function.hpp"
#include "neighbor_generator.hpp"
#include "search_strategy.hpp"

namespace arbitrator
{
    /**
     * \brief Implementation of PlanningStrategy using a generic tree search 
     *      algorithm
     * 
     * The fundamental components of this tree search are individually injected
     * into this class at construction time to allow for fine-tuning of the
     * algorithm and ensure better testability and separation of algorithmic 
     * concerns
     */
    class TreePlanner : public PlanningStrategy
    {
        public:
            /**
             * \brief Tree planner constructor
             * \param cf A reference to a CostFunction implementation
             * \param ng A reference to a NeighborGenerator implementation
             * \param ss A reference to a SearchStrategy implementation
             * \param target The desired duration of finished plans
             */
            TreePlanner(CostFunction &cf, 
                NeighborGenerator &ng, 
                SearchStrategy &ss, 
                ros::Duration target):
                cost_function_(cf),
                neighbor_generator_(ng),
                search_strategy_(ss),
                target_plan_duration_(target) {};

            /**
             * \brief Utilize the configured cost function, neighbor generator, 
             *      and search strategy, to generate a plan by means of tree search
             */
            cav_msgs::ManeuverPlan generate_plan();
        protected:
            CostFunction &cost_function_;
            NeighborGenerator &neighbor_generator_;
            SearchStrategy &search_strategy_;
            ros::Duration target_plan_duration_;
    };
};

#endif //__ARBITRATOR_INCLUDE_TREE_PLANNER_HPP__