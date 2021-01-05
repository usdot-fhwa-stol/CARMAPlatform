
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

/**
 * Test fixture for TrajectoryExecutor testing
 * Maintains publishers, subscribers, and message tracking for all tests.
 * State is reset between tests to ensure clean results.
 */

#ifndef __ARBITRATOR_INCLUDE_TEST_UTILS_HPP__
#define __ARBITRATOR_INCLUDE_TEST_UTILS_HPP__

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "arbitrator_state_machine.hpp"
#include "fixed_priority_cost_function.hpp"
#include "beam_search_strategy.hpp"
#include "capabilities_interface.hpp"

class ArbitratorStateMachineTest : public ::testing::Test 
{
    public:
        ArbitratorStateMachineTest() {};
        arbitrator::ArbitratorStateMachine sm_;
};

class FixedPriorityCostFunctionTest : public ::testing::Test 
{
    public:
        FixedPriorityCostFunctionTest():
            fpcf{{
                {"plugin_a", 10.0}, 
                {"plugin_b", 5.0}, 
                {"plugin_c", 15.0}}} {};
        arbitrator::FixedPriorityCostFunction fpcf;
};

class BeamSearchStrategyTest : public ::testing::Test 
{
    public:
        BeamSearchStrategyTest() {};
        arbitrator::BeamSearchStrategy bss{3};
};

#endif //__ARBITRATOR_INCLUDE_TEST_UTILS_HPP__
