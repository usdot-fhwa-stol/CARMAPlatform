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

#pragma once

#include <string>
#include <ros/ros.h>
#include <atomic>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/SetGuidanceActive.h>
#include <cav_srvs/SetEnableRobotic.h>
#include <std_msgs/Bool.h>
#include <cav_msgs/GuidanceState.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/RouteEvent.h>
#include "guidance/guidance_state_machine.hpp"

namespace guidance
{
    class GuidanceWorker
    {
        public:
            /*!
             * \brief Default constructor for GuidanceWorker
             */
            GuidanceWorker() = default;

            /*!
             * \brief Begin normal execution of Guidance worker. Will take over control flow of program and exit from here.
             * 
             * \return The exit status of this program
             */
            int run();

        private:
            
            // spin callback function
            bool spin_cb();

            // Message/service callbacks
            bool guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res);
            void route_event_cb(const cav_msgs::RouteEventConstPtr& msg);
            void robot_status_cb(const cav_msgs::RobotEnabledConstPtr& msg);
            void system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg);

            // Service servers 
            ros::ServiceServer guidance_activate_service_server_;

            // Publishers
            ros::Publisher state_publisher_;
            ros::ServiceClient enable_client_;

            // Subscribers
            ros::Subscriber robot_status_subscriber_;
            ros::Subscriber route_event_subscriber_;

            // Node handles
            ros::CARMANodeHandle nh_{}, pnh_{"~"};

            // Guidance state machine
            GuidanceStateMachine gsm_;
    };
}