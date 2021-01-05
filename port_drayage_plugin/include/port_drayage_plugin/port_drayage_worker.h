#pragma once

/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/MobilityOperation.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "port_drayage_plugin/port_drayage_state_machine.h"

namespace port_drayage_plugin
{
    /**
     * Implementation class for all the business logic of the PortDrayagePlugin
     * 
     * Should not contain any reference to ROS publishing/subscribing or params
     * and all of that data (as needed) should be pushed into it via methods. * This implementation uses lambdas and std::function to give it the ability
     * to publish the outbound MobilityOperation message w/o the need for it
     * to know about the ROS underlying implementation.
     */
    class PortDrayageWorker
    {
        private:
            cav_msgs::ManeuverPlanConstPtr _cur_plan;
            geometry_msgs::TwistStampedConstPtr _cur_speed;
            double _stop_speed_epsilon;
            PortDrayageStateMachine _pdsm;
            std::string _host_id;
            std::string _host_bsm_id;
            std::string _cmv_id;
            std::string _cargo_id;
            std::function<void(cav_msgs::MobilityOperation)> _publish_mobility_operation;

            // Constants
            const std::string PORT_DRAYAGE_PLUGIN_ID = "Port Drayage Plugin";
            const std::string PORT_DRAYAGE_STRATEGY_ID = "carma/port_drayage";
            const std::string PORT_DRAYAGE_ARRIVAL_OPERATION_ID = "ARRIVED_AT_DESTINATION";

        public:

            /**
             * \brief Standard constructor for the PortDrayageWorker
             * 
             * \param cmv_id The Carrier Motor Vehicle ID string for the host
             * vehicle
             * 
             * \param cargo_id The identification string for the cargo carried
             * by the host vehicle. If no cargo is being carried this should be
             * empty.
             * 
             * \param host_id The CARMA ID string for the host vehicle
             * 
             * \param mobility_operations_publisher A lambda containing the logic
             * necessary to publish a MobilityOperations message. This lambda should
             * contain all the necessary ROS logic so that it does not leak into
             * the implementation of this class
             * 
             * \param stop_speed_epsilon An epsilon factor to be used when
             * comparing the current vehicle's speed to 0.0
             */
            PortDrayageWorker(
                std::string cmv_id,
                std::string cargo_id,
                std::string host_id,
                std::function<void(cav_msgs::MobilityOperation)> mobility_operations_publisher, 
                double stop_speed_epsilon) :
                _cmv_id(cmv_id),
                _cargo_id(cargo_id),
                _host_id(host_id),
                _publish_mobility_operation(mobility_operations_publisher),
                _stop_speed_epsilon(stop_speed_epsilon) {};


            /**
             * \brief Initialize the PortDrayageWorker, setting up it's relation
             * to the state machine.
             */
            void initialize();

            /**
             * \brief Callback for usage by the PortDrayageStateMachine
             */
            void on_arrived_at_destination();

            /**
             * \brief Assemble the current dataset into a MobilityOperations
             * message with a JSON formatted body containing CMV ID and cargo ID
             */
            cav_msgs::MobilityOperation compose_arrival_message() const;

            /**
             * \brief Set the current plan from the arbitrator
             */
            void set_maneuver_plan(const cav_msgs::ManeuverPlanConstPtr& plan);

            /**
             * \brief Set the current speed as measured by the vehicle's sensors
             */
            void set_current_speed(const geometry_msgs::TwistStampedConstPtr& speed);

            /**
             * \brief Spin and process data
             */
            bool spin();
            
            /**
             * \brief Check to see if the vehicle has stopped under the command of the 
             * Port Drayage Plugin.
             * 
             * \param plan The current maneuver plan
             * \param speed The current vehicle speed
             * 
             * \return True if the vehicle has been stopped by the PDP, false o.w.
             */
            bool check_for_stop(const cav_msgs::ManeuverPlanConstPtr& plan, const geometry_msgs::TwistStampedConstPtr& speed) const;


    };
} // namespace port_drayage_plugin
