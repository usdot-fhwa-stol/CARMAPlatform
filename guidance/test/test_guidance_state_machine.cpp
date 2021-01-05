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

#include "guidance/guidance_state_machine.hpp"
#include "guidance/guidance_worker.hpp"
#include <gtest/gtest.h>

TEST(GuidanceStateMachineTest, testStates)
{
    guidance::GuidanceStateMachine gsm;
    // test initial state
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));
    // test SetEnableRobotic call flag
    EXPECT_TRUE(!gsm.shouldCallSetEnableRobotic());
    gsm.onSetGuidanceActive(true);
    // test engage on startup state, should not casue state change
    EXPECT_EQ(1, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::SystemAlert alert;
    alert.type = alert.DRIVERS_READY;
    cav_msgs::SystemAlertConstPtr alert_pointer(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer);
    // test drivers ready state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    // test active state
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    EXPECT_TRUE(gsm.shouldCallSetEnableRobotic());
    EXPECT_TRUE(!gsm.shouldCallSetEnableRobotic());
    gsm.onSetGuidanceActive(false);
    // test disengage on active
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    cav_msgs::RobotEnabled engage_status;
    engage_status.robot_enabled = true;
    engage_status.robot_active = true;
    cav_msgs::RobotEnabled disengage_status;
    disengage_status.robot_enabled = true;
    disengage_status.robot_active = false;
    cav_msgs::RobotEnabledConstPtr engage_status_pointer(new cav_msgs::RobotEnabled(engage_status));
    cav_msgs::RobotEnabledConstPtr disengage_status_pointer(new cav_msgs::RobotEnabled(disengage_status));
    gsm.onRoboticStatus(disengage_status_pointer);
    EXPECT_EQ(3, static_cast<int>(gsm.getCurrentState()));
    gsm.onRoboticStatus(engage_status_pointer);
    // test engaged state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onRoboticStatus(engage_status_pointer);
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(false);
    gsm.onRoboticStatus(disengage_status_pointer);
    // test disengage and restart state
    EXPECT_EQ(2, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    gsm.onRoboticStatus(engage_status_pointer);
    // test re-engage state
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    gsm.onRoboticStatus(disengage_status_pointer);
    // test manual override and inactive state
    EXPECT_EQ(5, static_cast<int>(gsm.getCurrentState()));
    gsm.onSetGuidanceActive(true);
    gsm.onRoboticStatus(engage_status_pointer);
    // test re-engage state from inactive
    EXPECT_EQ(4, static_cast<int>(gsm.getCurrentState()));
    alert.type = alert.SHUTDOWN;
    cav_msgs::SystemAlertConstPtr alert_pointer_2(new cav_msgs::SystemAlert(alert));
    gsm.onSystemAlert(alert_pointer_2);
    // test shut down state
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));
    // Should not recover from OFF state
    gsm.onSystemAlert(alert_pointer);
    EXPECT_EQ(0, static_cast<int>(gsm.getCurrentState()));
}

// Run all the tests
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
