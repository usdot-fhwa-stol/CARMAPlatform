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

#include "arbitrator_state_machine.hpp"
#include "test_utils.h"
#include <gtest/gtest.h>

namespace arbitrator
{
    TEST_F(ArbitratorStateMachineTest, testInitalize)
    {
        ASSERT_EQ(ArbitratorState::INITIAL, sm_.get_state());
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE));
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testPlanningCycle)
    {
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE));
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.get_state());
        ASSERT_EQ(ArbitratorState::WAITING, sm_.submit_event(ArbitratorEvent::PLANNING_COMPLETE));
        ASSERT_EQ(ArbitratorState::WAITING, sm_.get_state());
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.submit_event(ArbitratorEvent::PLANNING_TIMER_TRIGGER));
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testPauseResumeFromPlanning)
    {
        sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
        ASSERT_EQ(ArbitratorState::PAUSED, sm_.submit_event(ArbitratorEvent::ARBITRATOR_PAUSED));
        ASSERT_EQ(ArbitratorState::PAUSED, sm_.get_state());
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.submit_event(ArbitratorEvent::ARBITRATOR_RESUMED));
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testPauseResumeFromWaiting)
    {
        sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
        sm_.submit_event(ArbitratorEvent::PLANNING_COMPLETE);
        ASSERT_EQ(ArbitratorState::PAUSED, sm_.submit_event(ArbitratorEvent::ARBITRATOR_PAUSED));
        ASSERT_EQ(ArbitratorState::PAUSED, sm_.get_state());
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.submit_event(ArbitratorEvent::ARBITRATOR_RESUMED));
        ASSERT_EQ(ArbitratorState::PLANNING, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testShutdown1)
    {
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED));
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testShutdown2)
    {
        sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED));
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testShutdown3)
    {
        sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
        sm_.submit_event(ArbitratorEvent::PLANNING_COMPLETE);
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED));
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.get_state());
    }

    TEST_F(ArbitratorStateMachineTest, testShutdown4)
    {
        sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
        sm_.submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED));
        ASSERT_EQ(ArbitratorState::SHUTDOWN, sm_.get_state());
    }
}
