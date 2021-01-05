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

#ifndef __ARBITRATOR_INCLUDE_ARBITRATOR_HPP__
#define __ARBITRATOR_INCLUDE_ARBITRATOR_HPP__

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include "arbitrator_state_machine.hpp"
#include "planning_strategy.hpp"
#include "capabilities_interface.hpp"
#include <cav_msgs/GuidanceState.h>

namespace arbitrator 
{
    /**
     * Primary work class for the Arbitrator package
     * 
     * Governs the interactions of plugins during the maneuver planning phase of
     * the CARMA planning process. Utilizes a generic planning interface to allow
     * for reconfiguration with other paradigms in the future.
     */
    class Arbitrator 
    {
        public:
            /**
             * \brief Constructor for arbitrator class taking in dependencies via dependency injection
             * \param nh A CARMANodeHandle instance with a globally referenced ("/") path
             * \param pnh A CARMANodeHandle instance with a privately referenced ("~") path
             * \param sm An ArbitratorStateMachine instance for regulating the states of the Arbitrator
             * \param ci A CapabilitiesInterface for querying plugins
             * \param planning_strategy A planning strategy implementation for generating plans
             * \param min_plan_duration The minimum acceptable length of a plan
             * \param planning_frequency The frequency at which to generate high-level plans when engaged
             */ 
            Arbitrator(ros::CARMANodeHandle *nh, 
                ros::CARMANodeHandle *pnh, 
                ArbitratorStateMachine *sm, 
                CapabilitiesInterface *ci, 
                PlanningStrategy &planning_strategy,
                ros::Duration min_plan_duration,
                ros::Rate planning_frequency):
                sm_(sm),
                nh_(nh),
                pnh_(pnh),
                capabilities_interface_(ci),
                planning_strategy_(planning_strategy),
                initialized_(false),
                min_plan_duration_(min_plan_duration),
                time_between_plans_(planning_frequency.expectedCycleTime()) {};
            
            /**
             * \brief Begin the operation of the arbitrator.
             * 
             * Loops internally via ros::Duration sleeps and spins
             */
            void run();
        protected:
            /**
             * \brief Function to be executed during the initial state of the Arbitrator
             */
            void initial_state();

            /**
             * \brief Function to be called when the Arbitrator begins planning
             */
            void planning_state();

            /**
             * \brief Function to be executed when the Arbitrator has finished planning
             * and is awaiting another planning cycle
             */
            void waiting_state();

            /**
             * \brief Function to be executed when the Arbitrator is not planning but also
             * not awaiting a new plan cycle
             */
            void paused_state();

            /**
             * \brief Function to be executed when the Arbitrator is to clean up and shutdown
             */
            void shutdown_state();

            /**
             * \brief Callback for receiving Guidance state machine updates
             * \param msg The new GuidanceState message
             */
            void guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg);

        private:
            ArbitratorStateMachine *sm_;
            ros::Publisher final_plan_pub_;
            ros::Subscriber guidance_state_sub_;
            ros::CARMANodeHandle *nh_;
            ros::CARMANodeHandle *pnh_;
            ros::Duration min_plan_duration_;
            ros::Duration time_between_plans_;
            ros::Time next_planning_process_start_;
            CapabilitiesInterface *capabilities_interface_;
            PlanningStrategy &planning_strategy_;
            bool initialized_;
    };
};

#endif
