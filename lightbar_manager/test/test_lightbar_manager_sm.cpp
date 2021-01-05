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

#include "lightbar_manager/lightbar_manager_sm.hpp"
#include <cav_msgs/GuidanceState.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace lightbar_manager
{

TEST(LightBarManagerStateMachineTest, testStates)
{
    LightBarManagerStateMachine lbsm, lbsm_reset;
    // test initial state
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    
    /*
    * test possible/impossible states from lightbar DISENGAGE
    */
    cav_msgs::GuidanceStatePtr msg_ptr = boost::make_shared<cav_msgs::GuidanceState>();
    // guidance startup, lightbar should not change state
    msg_ptr->state = cav_msgs::GuidanceState::STARTUP;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    // guidance driver_ready, lightbar should become active
    msg_ptr->state = cav_msgs::GuidanceState::DRIVERS_READY;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance SHUTDOWN, lightbar should disengage
    msg_ptr->state = cav_msgs::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
     // guidance INACTIVE, lightbar should become active
    msg_ptr->state = cav_msgs::GuidanceState::INACTIVE;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance ACTIVE, lightbar should change to ACTIVE
    msg_ptr->state = cav_msgs::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    lbsm = lbsm_reset;
    // guidance ENGAGED, lightbar should change to ENGAGED
    msg_ptr->state = cav_msgs::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());

    /*
    * test possible/impossible states from ENGAGED/ACTIVE
    */
    msg_ptr->state = cav_msgs::GuidanceState::ENGAGED;
    // guidance ENGAGED, lightbar should not change
    msg_ptr->state = cav_msgs::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());
    // guidance ACTIVE, lightbar should change to ACTIVE
    msg_ptr->state = cav_msgs::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ACTIVE, lbsm.getCurrentState());
    // guidance ENGAGED, lightbar should change to ENGAGED
    msg_ptr->state = cav_msgs::GuidanceState::ENGAGED;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(ENGAGED, lbsm.getCurrentState());
    // guidance SHUTDOWN, lightbar should change to DISENGAGED
    msg_ptr->state = cav_msgs::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
    msg_ptr->state = cav_msgs::GuidanceState::ACTIVE;
    lbsm.handleStateChange(msg_ptr);
    // guidance SHUTDOWN, lightbar should change to DISENGAGED
    msg_ptr->state = cav_msgs::GuidanceState::SHUTDOWN;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());

    // Illogical states check, should not change the states.
    msg_ptr->state = 7;
    lbsm.handleStateChange(msg_ptr);
    EXPECT_EQ(DISENGAGED, lbsm.getCurrentState());
}

}   // namespace lightbar_manager

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
