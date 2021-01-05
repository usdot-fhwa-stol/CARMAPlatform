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

#include <cav_msgs/SystemAlert.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/GuidanceState.h>
#include <cav_msgs/RouteEvent.h>

namespace guidance
{
    class GuidanceStateMachine
    {
        public:
        
            enum Signal
            {
                INITIALIZED = 0,
                ACTIVATED = 1,
                ENGAGE = 2,
                DISENGAGED = 3,
                SHUTDOWN = 4,
                OVERRIDE = 5,
            };

            enum State
            {
                OFF = 0,
                STARTUP = 1,
                DRIVERS_READY = 2,
                ACTIVE = 3,
                ENGAGED = 4,
                INACTIVE = 5,
            };

            /*!
             * \brief Default constructor for GuidanceStateMachine
             */
            GuidanceStateMachine() = default;

            /*!
             * \brief Handle system_alert message from ROS network.
             */
            void onSystemAlert(const cav_msgs::SystemAlertConstPtr& msg);

            /*!
             * \brief Handle set_guidance_active service call from ROS network.
             */
            void onSetGuidanceActive(bool msg);

            /*!
             * \brief Handle robotic_status message from ROS network.
             */
            void onRoboticStatus(const cav_msgs::RobotEnabledConstPtr& msg);

            /*!
             * \brief Handle route event message.
             */
            void onRouteEvent(const cav_msgs::RouteEventConstPtr& msg);

            /*!
             * \brief Indicate if SetEnableRobotic needs to be called in ACTIVE state.
             */
            bool shouldCallSetEnableRobotic();

            /*!
             * \brief Get current state machine status.
             */
            uint8_t getCurrentState();

        private:

            /*!
             * \brief Actual state machine logic driven by signal enum.
             */
            void onGuidanceSignal(Signal signal);

            // a local variable keeps the current state machine state
            State current_guidance_state_ {State::STARTUP};

            // Previous robotic active status
            bool robotic_active_status_{false};

            // make one service call in ACTIVE state to engage
            bool called_robotic_engage_in_active_{false};
    };

}