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

#include <functional>

namespace port_drayage_plugin
{
    /**
     * \brief Enum containing the possible state values for the PortDrayagePlugin
     * 
     * INACTIVE = PortDrayagePlugin is not currently operating in an active
     *  capacity
     * EN_ROUTE = The vehicle is currently EN_ROUTE to a destination, either
     *  under the direciton of the PortDrayagePlugin or otherwise.
     * AWAITING_DIRECTION = The vehicle is currently stopped under the command
     *  of the PortDrayagePlugin and is awaiting further guidance from the 
     *  port infrastructure to tell it the next destination.
     */ 
    enum PortDrayageState
    {
        INACTIVE,
        EN_ROUTE,
        AWAITING_DIRECTION
    };

    /**
     * \brief Enum containing the event values that cause changes in the PortDrayagePlugin
     * state machine.
     * 
     * DRAYAGE_START = Drayage operations have been commenced by the vehicle
     * RECEIVED_NEW_DESTINATION = The port has commanded the vehicle to travel
     *   to a new destination.
     * ARRIVED_AT_DESTINATION = The vehicle has arrived at it's destination.
     * DRAYAGE_COMPLETED = The drayage operation is complete and the vehicle has
     *  exited the port area.
     */
    enum PortDrayageEvent
    {
        DRAYAGE_START,
        RECEIVED_NEW_DESTINATION,
        ARRIVED_AT_DESTINATION,
        DRAYAGE_COMPLETED
    };

    /**
     * \Brief An implementation of the state machine for the PortDrayagePlugin.
     * 
     * The state machine accepts events as inputs which drive the internal state
     * changes. This state can be queried at any given point in time, but also
     * each transition into a state has an associated callback which will be 
     * invoked when that state is entered.
     */
    class PortDrayageStateMachine
    {
        private:
            PortDrayageState _state;
            std::function<void()> _on_system_startup;
            std::function<void()> _on_received_new_destination;
            std::function<void()> _on_arrived_at_destination;
            std::function<void()> _on_drayage_completed;

        public:
            /**
             * \brief Constructor the PortDrayageStateMachine.
             * 
             * The system initially starts in INACTIVE state.
             */
            PortDrayageStateMachine() :
                _state(PortDrayageState::INACTIVE) {};
        
            /**
             * \Brief Inform the state machine that an event has transpired.
             * 
             * \param event The event that took place.
             */
            void process_event(PortDrayageEvent event);

            /**
             * \brief Get the current state of the state machine
             * \return The current state value
             */
            const PortDrayageState get_state();

            /**
             * \brief Set the callback to be invoked upon transitioning out of the
             * inactive state.
             * \param cb The std::function callback object
             */
            void set_on_system_startup_callback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the 
             * EN_ROUTE state
             * \param cb The std::function callback object
             */
            void set_on_received_new_destination_callback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the 
             * AWAITING_DESTINATION state
             * \param cb The std::function callback object
             */
            void set_on_arrived_at_destination_callback(const std::function<void()> &cb);

            /**
             * \brief Set the callback to be invoked upon transitioning into the
             * inactive state.
             * \param cb The std::function callback object
             */
            void set_on_drayage_completed_callback(const std::function<void()> &cb);
    };
} // namespace port_drayage_plugin

