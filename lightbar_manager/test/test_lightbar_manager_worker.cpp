/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either exp`tations under
 * the License.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cav_msgs/GuidanceState.h>
#include <thread>
#include "lightbar_manager/lightbar_manager_node.hpp"
#include "lightbar_manager/lightbar_manager_worker.hpp"

namespace lightbar_manager
{


TEST(LightBarManagerWorkerTest, testRequestControl) 
{
    LightBarManagerWorker worker("lightbar_manager");
    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    worker.setIndicatorControllers();
    worker.control_priorities.push_back("lightbar_manager");
    worker.control_priorities.push_back("tester1");
    worker.control_priorities.push_back("tester2");
    worker.control_priorities.push_back("tester3");

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        worker.light_status.push_back(OFF);

    std::vector<LightBarIndicator> greens_default = {GREEN_SOLID, GREEN_FLASH};
    worker.requestControl(greens_default, "lightbar_manager");

    std::map<LightBarIndicator, std::string> curr_owners = worker.getIndicatorControllers();
    std::vector<LightBarIndicator> exclusive_list = {YELLOW_DIM, YELLOW_SIDES}, 
        greens = {GREEN_FLASH, GREEN_SOLID},
        curr_ind,
        denied_list;
    
    // Green indicators are for lightbar_manager only, and should have been taken control of already
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_SOLID]);
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_FLASH]);

    // Anyone that is not in the priority list should be treated with lowest priority
    // therefore, even if requester and controller are both not in the list, requester cannot control it
    denied_list = worker.requestControl(exclusive_list, "CarmaIsAwesome!");
    EXPECT_EQ(0, denied_list.size());
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ("CarmaIsAwesome!", curr_owners[YELLOW_DIM]);
    EXPECT_EQ("CarmaIsAwesome!", curr_owners[YELLOW_SIDES]);
    denied_list = worker.requestControl(exclusive_list, "CarmaIsGreat!");
    EXPECT_EQ(2, denied_list.size());
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ("CarmaIsAwesome!", curr_owners[YELLOW_DIM]);
    EXPECT_EQ("CarmaIsAwesome!", curr_owners[YELLOW_SIDES]);
    
    /*
    *   Handle mutually exclusive cases
    */
    // Any plugin in the priority list should be able to take control of both indicators
    denied_list = worker.requestControl(exclusive_list, "tester3");
    EXPECT_EQ(0, denied_list.size());
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ("tester3", curr_owners[YELLOW_DIM]);
    EXPECT_EQ("tester3", curr_owners[YELLOW_SIDES]);

    // Anyone that is not in the priority list should be treated with lowest priority
    denied_list = worker.requestControl(exclusive_list, "CarmaIsAwesome!");
    EXPECT_EQ(2, denied_list.size());
    
    // Tester with higher priority should be able to take control of both
    denied_list = worker.requestControl(exclusive_list, "tester2");
    
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ("tester2", curr_owners[YELLOW_DIM]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_SIDES]);
    
    // Tester with lower priority should NOT be able to take control
    denied_list = worker.requestControl(exclusive_list, "tester3");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(2, denied_list.size());
    EXPECT_EQ("tester2", curr_owners[YELLOW_DIM]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_SIDES]);
    
    denied_list = worker.requestControl(greens, "tester1");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(2, denied_list.size());
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_SOLID]);
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_FLASH]);

    /* 
    * Handle mutually inexclusive indicators
    */

    // Taking control of left (or right) only should take control of flash and out too
    // as they are mutually inexclusive indicators
    curr_ind = {YELLOW_ARROW_LEFT};
    denied_list = worker.requestControl(curr_ind, "tester2");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(0, denied_list.size());
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_RIGHT]);

    // However, taking control of right now with lower priority should 
    // fail to control flash and out (but succeed for right)
    curr_ind = {YELLOW_ARROW_RIGHT};
    denied_list = worker.requestControl(curr_ind, "tester3");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(0, denied_list.size());
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("tester3", curr_owners[YELLOW_ARROW_RIGHT]);

    // On the other hand, taking control of right with higher priority than both
    // should succeed in control all three of them
    curr_ind = {YELLOW_ARROW_RIGHT};
    denied_list = worker.requestControl(curr_ind, "tester1");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(0, denied_list.size());
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_RIGHT]);

    // Taking control of flash or arrow_out should overwrite left and right as well
    curr_ind = {YELLOW_ARROW_OUT};
    denied_list = worker.requestControl(curr_ind, "lightbar_manager");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(0, denied_list.size());
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_RIGHT]);
    
    // Taking control of indicator that no one explicitly requested
    // should still fail if this indicator is mutually inexculsive from other indicator with higher priority
    curr_ind = {YELLOW_FLASH};
    denied_list = worker.requestControl(curr_ind, "tester2");
    curr_owners= worker.getIndicatorControllers();
    EXPECT_EQ(1, denied_list.size());
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("lightbar_manager", curr_owners[YELLOW_ARROW_RIGHT]);
    
}

TEST(LightBarManagerWorkerTest, testReleaseControl) 
{
    LightBarManagerWorker worker("lightbar_manager");
    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    worker.setIndicatorControllers();
    worker.control_priorities.push_back("lightbar_manager");
    worker.control_priorities.push_back("tester1");
    worker.control_priorities.push_back("tester2");
    worker.control_priorities.push_back("tester3");

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        worker.light_status.push_back(OFF);

    std::vector<LightBarIndicator> greens_default = {GREEN_SOLID, GREEN_FLASH};
    worker.requestControl(greens_default, "lightbar_manager");

    std::map<LightBarIndicator, std::string> curr_owners = worker.getIndicatorControllers();
    std::vector<LightBarIndicator> greens = {GREEN_FLASH, GREEN_SOLID}, curr_ind;

    // if it is not currently owning the light, it should not remove control
    worker.releaseControl(greens, "tester1");
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_FLASH]);
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_SOLID]);

    // It should release all mutually inexclusive indicators indirectly at the same time
    curr_ind = {YELLOW_ARROW_OUT};
    worker.requestControl(curr_ind, "tester3");
    curr_owners = worker.getIndicatorControllers();
    EXPECT_EQ("tester3", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester3", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester3", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("tester3", curr_owners[YELLOW_ARROW_RIGHT]);
    worker.releaseControl(curr_ind, "tester3");
    curr_owners = worker.getIndicatorControllers();
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_RIGHT]);

    // It should also release all mutually inexclusive indicators indirectly
    // ONLY if it has higher priority than the component controlling it.
    curr_ind = {YELLOW_ARROW_LEFT};
    worker.requestControl(curr_ind, "tester3");
    curr_ind = {YELLOW_ARROW_RIGHT};
    worker.requestControl(curr_ind, "tester2");
    curr_owners = worker.getIndicatorControllers();
    EXPECT_EQ("tester3", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_RIGHT]);
    curr_ind = {YELLOW_ARROW_LEFT};
    worker.releaseControl(curr_ind, "tester3");
    curr_owners = worker.getIndicatorControllers();
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_FLASH]);
    EXPECT_EQ("tester2", curr_owners[YELLOW_ARROW_RIGHT]);
}

TEST(LightBarManagerWorkerTest, testSetIndicator) 
{
    LightBarManagerWorker worker("lightbar_manager");
    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    worker.setIndicatorControllers();
    worker.control_priorities.push_back("lightbar_manager");
    worker.control_priorities.push_back("tester1");
    worker.control_priorities.push_back("tester2");
    worker.control_priorities.push_back("tester3");

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        worker.light_status.push_back(OFF);

    std::vector<LightBarIndicator> greens_default = {GREEN_SOLID, GREEN_FLASH};
    worker.requestControl(greens_default, "lightbar_manager");


    std::vector<IndicatorStatus> correct_light_status;
    std::vector<LightBarIndicator> target_indicators;
    // Lightbar_manager should be able to set the green indicators right away
    //target_indicators = {GREEN_SOLID};
    //worker.requestControl(target_indicators, "tester3");
    std::map<LightBarIndicator, std::string> curr_owners = worker.getIndicatorControllers();
    EXPECT_EQ("lightbar_manager", curr_owners[GREEN_SOLID]);
    worker.light_status = worker.setIndicator(GREEN_SOLID, ON, "lightbar_manager");
    correct_light_status = {ON, OFF, OFF, OFF, OFF, OFF, OFF, OFF};
    EXPECT_EQ(correct_light_status, worker.light_status);
    
    /*
    *   Handle mutually inexclusive indicators
    */
    // LightbarManager changing one green indicator should change the other too
    worker.light_status = worker.setIndicator(GREEN_FLASH, ON, "lightbar_manager");
    correct_light_status = {OFF, ON, OFF, OFF, OFF, OFF, OFF, OFF};
    EXPECT_EQ(correct_light_status, worker.light_status);
    // However changing the same indicator to same status should not change anything
    worker.light_status = worker.setIndicator(GREEN_FLASH, ON, "lightbar_manager");
    correct_light_status = {OFF, ON, OFF, OFF, OFF, OFF, OFF, OFF};
    EXPECT_EQ(correct_light_status, worker.light_status);
    // Unlike request/release control func, this should only turn ON what is requested
    target_indicators = {YELLOW_ARROW_LEFT};
    worker.requestControl(target_indicators, "tester3");
    worker.light_status = worker.setIndicator(YELLOW_ARROW_LEFT, ON, "tester3");
    correct_light_status = {OFF, ON, OFF, OFF, OFF, ON, OFF, OFF};
    EXPECT_EQ(correct_light_status, worker.light_status);

}

TEST(LightBarManagerWorkerTest, testHasHigherPriority) 
{
    LightBarManagerWorker worker("lightbar_manager");
    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    worker.setIndicatorControllers();
    worker.control_priorities.push_back("lightbar_manager");
    worker.control_priorities.push_back("tester1");
    worker.control_priorities.push_back("tester2");
    worker.control_priorities.push_back("tester3");

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        worker.light_status.push_back(OFF);

    std::vector<LightBarIndicator> greens_default = {GREEN_SOLID, GREEN_FLASH};
    worker.requestControl(greens_default, "lightbar_manager");


    // Lightbar compared to anything should be higher
    EXPECT_EQ(true, worker.hasHigherPriority("lightbar_manager", "tester1"));
    // Test priority list
    EXPECT_EQ(false, worker.hasHigherPriority("tester2", "tester1"));
    EXPECT_EQ(true, worker.hasHigherPriority("tester1", "tester3"));
    // Component not in the priority list should be treated as the lowest
    EXPECT_EQ(true, worker.hasHigherPriority("tester3", "CARMAMakesWorldBetter"));
    // If both components are not in the list, the new requester has lower priority
    EXPECT_EQ(false, worker.hasHigherPriority("CARMAAtItAgain", "CARMAMakesWorldBetter"));
}

TEST(LightBarManagerWorkerTest, testGetLightBarStatusMsg) 
{
    LightBarManagerWorker worker("lightbar_manager");
    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    worker.setIndicatorControllers();
    worker.control_priorities.push_back("lightbar_manager");
    worker.control_priorities.push_back("tester1");
    worker.control_priorities.push_back("tester2");
    worker.control_priorities.push_back("tester3");

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        worker.light_status.push_back(OFF);

    std::vector<LightBarIndicator> greens_default = {GREEN_SOLID, GREEN_FLASH};
    worker.requestControl(greens_default, "lightbar_manager");


    std::vector<IndicatorStatus> all_indicators = {ON, OFF, ON, ON, OFF, OFF, OFF, ON};
    cav_msgs::LightBarStatus msg = worker.getLightBarStatusMsg(all_indicators);
    EXPECT_EQ(cav_msgs::LightBarStatus::ON, msg.green_solid);
    EXPECT_EQ(cav_msgs::LightBarStatus::OFF, msg.green_flash);
    EXPECT_EQ(cav_msgs::LightBarStatus::ON, msg.sides_solid);
    EXPECT_EQ(cav_msgs::LightBarStatus::ON, msg.yellow_solid);
    EXPECT_EQ(cav_msgs::LightBarStatus::OFF, msg.flash);
    EXPECT_EQ(cav_msgs::LightBarStatus::ON, msg.left_arrow);
    EXPECT_EQ(cav_msgs::LightBarStatus::ON, msg.right_arrow);
}

TEST(LightBarManagerWorkerTest, testHandleHandleTurnSignal) 
{
    // Handle left/right indicators with arrow_out correctly
    LightBarManagerWorker worker("lightbar_manager");

    automotive_platform_msgs::TurnSignalCommandPtr turn_signal = boost::make_shared<automotive_platform_msgs::TurnSignalCommand>();
    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
    auto light_signals = worker.handleTurnSignal(turn_signal);

    EXPECT_EQ(light_signals.size(), 1);
    EXPECT_EQ(light_signals[0], lightbar_manager::LightBarIndicator::YELLOW_ARROW_RIGHT);
    
    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    light_signals = worker.handleTurnSignal(turn_signal);

    EXPECT_EQ(light_signals.size(), 1);
    EXPECT_EQ(light_signals[0], lightbar_manager::LightBarIndicator::YELLOW_ARROW_RIGHT);

    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    light_signals = worker.handleTurnSignal(turn_signal);
    EXPECT_EQ(light_signals.size(), 0);

    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
    light_signals = worker.handleTurnSignal(turn_signal);
    EXPECT_EQ(light_signals.size(), 1);
    EXPECT_EQ(light_signals[0], lightbar_manager::LightBarIndicator::YELLOW_ARROW_LEFT);
    
    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    light_signals = worker.handleTurnSignal(turn_signal);
    EXPECT_EQ(light_signals.size(), 1);
    EXPECT_EQ(light_signals[0], lightbar_manager::LightBarIndicator::YELLOW_ARROW_LEFT);

    turn_signal->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    light_signals = worker.handleTurnSignal(turn_signal);
    EXPECT_EQ(light_signals.size(), 0);
}


} // namespace lightbar_manager
