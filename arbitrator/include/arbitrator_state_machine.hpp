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

#ifndef __ARBITRATOR_INCLUDE_ARBITRATOR_STATE_MACHINE_HPP__
#define __ARBITRATOR_INCLUDE_ARBITRATOR_STATE_MACHINE_HPP__

#include <vector>

namespace arbitrator
{
    /**
     * Possible states for the Arbitrator to be in:
     * INITIAL- Startup state while the rest of the system is initalizing
     * PLANNING - Actiively generating a maneuver plan
     * WAITING - After the completion of a plan, waiting for a timer trigger to replan
     * PAUSED - Similar to waiting, but will ignore a timer trigger. Must be resumed externally.
     */
    enum ArbitratorState {
        INITIAL = 0,
        PLANNING,
        WAITING,
        PAUSED,
        SHUTDOWN
    };

    /**
     * Possible events for the Arbitrator to respond to:
     * SYSTEM_STARTUP_COMPLETE - The rest of the system is ready for the arbitrator to being planning
     * PLANNING_COMPLETE - The Arbitrator itself has completed a maneuver plan
     * PLANNING_TIMER_TRIGGER - The time has come for another plan to be generated
     * ARBITRATOR_PAUSED - The Arbitrator has been paused externally
     * ARBITRATOR_RESUMED - The Arbitrator has been resumed externally
     */
    enum ArbitratorEvent {
        SYSTEM_STARTUP_COMPLETE = 0,
        PLANNING_COMPLETE,
        PLANNING_TIMER_TRIGGER,
        ARBITRATOR_PAUSED,
        ARBITRATOR_RESUMED,
        SYSTEM_SHUTDOWN_INITIATED
    };

    /**
     * 3-tuple for describing legal state transitions
     */
    struct ArbitratorStateTransition {
        ArbitratorState current_state;
        ArbitratorEvent input_event;
        ArbitratorState final_state;
    };


    /**
     * The ArbitratorStateMachine class is responsible for regulating the state
     * transitions within the Arbitrator node. It processess events and
     * correlates them against a list of legal transitions to generate the next
     * state the Arbitrator will take. The transition list is defined internally
     * and each transition must be unique/deterministic (each state/event pair
     * must transition to one-and-only-one other state) or behavior is undefined.
     */
    class ArbitratorStateMachine
    {
        public:
            ArbitratorStateMachine() : 
                current_state(INITIAL){};
            /**
             * Submit an event for evaluation by the Arbitrator state machine.
             * @param The event to process
             * @return The state of the Arbitrator after the event has been processed.
             */
            ArbitratorState submit_event(ArbitratorEvent event);

            /**
             * Get the current state of the Arbitrator
             * @return The state of the Arbitrator
             */
            ArbitratorState get_state();

        private:
            /**
             * Private, fixed state transition table
             */
            const std::vector<ArbitratorStateTransition> ARBITRATOR_TRANSITIONS =
            {   
                // Nominal arbitrator functionality
                {INITIAL, SYSTEM_STARTUP_COMPLETE, PLANNING},
                {PLANNING, PLANNING_COMPLETE, WAITING},
                {WAITING, PLANNING_TIMER_TRIGGER, PLANNING},

                // Interrupt and resume
                {PLANNING, ARBITRATOR_PAUSED, PAUSED},
                {WAITING, ARBITRATOR_PAUSED, PAUSED},
                {PAUSED, ARBITRATOR_RESUMED, PLANNING},

                // System shutdown procedures
                {INITIAL, SYSTEM_SHUTDOWN_INITIATED, SHUTDOWN},
                {PLANNING, SYSTEM_SHUTDOWN_INITIATED, SHUTDOWN},
                {WAITING, SYSTEM_SHUTDOWN_INITIATED, SHUTDOWN},
                {PAUSED, SYSTEM_SHUTDOWN_INITIATED, SHUTDOWN},
            };

            ArbitratorState current_state;
    };
}

#endif
