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
#include <novatel_gps_msgs/Inspva.h>
#include <boost/optional.hpp>

#include "port_drayage_plugin/port_drayage_state_machine.h"

namespace port_drayage_plugin
{
    /**
     * Convenience struct for storing all data contained in a received MobilityOperation message's
     * strategy_params field with strategy "carma/port_drayage"
     */
    struct PortDrayageMobilityOperationMsg
    {
        boost::optional<std::string> cargo_id;
        std::string operation;
        PortDrayageEvent port_drayage_event_type; // PortDrayageEvent associated with this message
        bool has_cargo; // Flag to indicate whether vehicle has cargo during this action
        boost::optional<std::string> current_action_id; // Identifier for the action this message is related to
        boost::optional<double> dest_longitude;  // Destination longitude for the carma vehicle
        boost::optional<double> dest_latitude;   // Destination latitude for the carma vehicle
        boost::optional<double> start_longitude; // Starting longitude of the carma vehicle
        boost::optional<double> start_latitude;  // Starting latitude of the carma vehicle
    };

    /**
     * Convenience struct for storing the vehicle's current latitude/longitude coordinates
     */
    struct LatLonCoordinate
    {
        double latitude;
        double longitude;
    };

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
            unsigned long _cmv_id;
            std::string _cargo_id;
            std::function<void(cav_msgs::MobilityOperation)> _publish_mobility_operation;
            bool _has_cargo;

            // Data member for storing the strategy_params field of the last processed port drayage MobilityOperation message intended for this vehicle's cmv_id
            std::string _previous_strategy_params;

            // Constants
            const std::string PORT_DRAYAGE_PLUGIN_ID = "Port Drayage Plugin";
            const std::string PORT_DRAYAGE_STRATEGY_ID = "carma/port_drayage";
            const std::string PORT_DRAYAGE_ARRIVAL_OPERATION_ID = "ARRIVED_AT_DESTINATION";

        public:

            /**
             * \brief Standard constructor for the PortDrayageWorker
             * 
             * \param cmv_id The Carrier Motor Vehicle ID (an unsigned long) for the host
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
                unsigned long cmv_id,
                std::string cargo_id,
                std::string host_id,
                std::function<void(cav_msgs::MobilityOperation)> mobility_operations_publisher, 
                double stop_speed_epsilon) :
                _cmv_id(cmv_id),
                _cargo_id(cargo_id),
                _host_id(host_id),
                _publish_mobility_operation(mobility_operations_publisher),
                _stop_speed_epsilon(stop_speed_epsilon) {
                    _has_cargo = (_cargo_id == "") ? false : true;
                };


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
             * \brief Callback to process a received MobilityOperation message
             * \param mobility_operation_msg a received MobilityOperation message
             */
            void on_inbound_mobility_operation(const cav_msgs::MobilityOperationConstPtr& mobility_operation_msg);

            /**
             * \brief Function to help parse the text included in an inbound MobilityOperation message's 
             *  strategy_params field according to the JSON schema intended for MobilityOperation messages
             *  with strategy type 'carma/port_drayage'. Stores the parsed information in _latest_mobility_operation_msg.
             * \param mobility_operation_strategy_params the strategy_params field of a MobilityOperation message
             */
            void mobility_operation_message_parser(std::string mobility_operation_strategy_params);

            /**
             * \brief Callback to store the host vehicle's current latitude/longitude coordinates 
             */
            void set_current_gps_position(const novatel_gps_msgs::InspvaConstPtr& gps_position);

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

            // PortDrayageMobilityOperationMsg object for storing strategy_params data of a received port drayage MobilityOperation message intended for this vehicle's cmv_id
            PortDrayageMobilityOperationMsg _latest_mobility_operation_msg;

            // LatLonCoordinate object for storing the vehicle's current gps latitude/longitude coordinates
            LatLonCoordinate _current_gps_position;

    };
} // namespace port_drayage_plugin
