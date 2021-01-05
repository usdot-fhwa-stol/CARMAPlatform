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

#ifndef __ARBITRATOR_INCLUDE_COST_SYSTEM_COST_FUNCTION_HPP__
#define __ARBITRATOR_INCLUDE_COST_SYSTEM_COST_FUNCTION_HPP__

#include <ros/ros.h>
#include "cost_function.hpp"
#include <map>
#include <string>

namespace arbitrator
{
    /**
     * \brief Implementation of the CostFunction interface
     * 
     * Implements costs by utilizing a ROS service call to the CARMA cost plugin
     * system. Passes on the input maneuver plan to the Cost Plugin System for 
     * computation of the actual cost and cost per unit distance.
     */
    class CostSystemCostFunction : public CostFunction
    {
        public:
            /**
             * \brief Constructor for FixedPriorityCostFunction
             */
            CostSystemCostFunction() {};

            /**
             * Initialize the CostSystemCostFunction to communicate over the network.
             * Sets up any ROS service clients needed to interact with the cost plugin system.
             * 
             * Must be called before using this cost function implementation.
             * 
             * \param nh A publicly namespaced nodehandle
             */
            void init(ros::NodeHandle &nh);

            /**
             * \brief Compute the unit cost over distance of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost divided by the total distance of the plan
             * 
             * \throws std::logic_error if not initialized
             */
            double compute_total_cost(const cav_msgs::ManeuverPlan& plan);

            /**
             * \brief Compute the unit cost over distance of a given maneuver plan
             * \param plan The plan to evaluate
             * \return double The total cost divided by the total distance of the plan
             * \throws std::logic_error if not initialized
             */
            double compute_cost_per_unit_distance(const cav_msgs::ManeuverPlan& plan);
        private:
            ros::ServiceClient cost_system_sc_;
            bool initialized_ = false;
    };
};

#endif //__ARBITRATOR_INCLUDE_COST_SYSTEM_COST_FUNCTION_HPP__

