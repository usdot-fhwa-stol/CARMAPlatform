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

namespace lightbar_manager
{

TEST(LightBarManagerNodeTest, testSetIndicator) 
{
    LightBarManager node("lightbar_manager");
    // initialize worker that is unit testable
    node.init("test");
    int response_code;
    /*
    * Response_code: SUCCESS(0) will never be achieved in unit test
    * However, receiving ERROR(2) can be thought as successful as it reached 
    * lightbar driver service request stage
    */

    // Lightbar_manager should be able to set the green indicators right away  
    response_code = node.setIndicator(GREEN_SOLID, ON, "lightbar_manager");
    EXPECT_EQ(2, response_code);
    // Any other component should not be able to change indicators
    response_code = node.setIndicator(GREEN_SOLID, ON, "CARMAAtItAgain!");
    EXPECT_EQ(1, response_code);
    // Empty component should not be able to control as well
    // Although they are technically controlling it
    response_code = node.setIndicator(YELLOW_ARROW_LEFT, ON, "");
    EXPECT_EQ(1, response_code); 
    // Priority list component cannot change indicator before taking control
    response_code = node.setIndicator(YELLOW_ARROW_LEFT, ON, "tester1");
    EXPECT_EQ(1, response_code);

}

TEST(LightBarManagerNodeTest, testTurnOffAll) 
{
    LightBarManager node("lightbar_manager");
    // initialize worker that is unit testable
    node.init("test");
    auto worker = node.getWorker();

    // As unit test cannot actually turn on indicators, observing change in control is enough
    std::vector<LightBarIndicator> some_indicators = 
        {YELLOW_ARROW_LEFT, YELLOW_ARROW_OUT, YELLOW_DIM};
    worker->requestControl(some_indicators, "tester1");
    std::map<LightBarIndicator, std::string> curr_owners= worker->getIndicatorControllers();
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("tester1", curr_owners[YELLOW_DIM]);
    // In order to turn all off, lightbar_manager has to take control and release once done
    node.turnOffAll();
    curr_owners= node.getWorker()->getIndicatorControllers();
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_LEFT]);
    EXPECT_EQ("", curr_owners[YELLOW_ARROW_OUT]);
    EXPECT_EQ("", curr_owners[YELLOW_DIM]);
}

TEST(LightBarManagerNodeTest, testTurnSignalCallback)
{
    LightBarManager node("lightbar_manager");
    // initialize worker that is unit testable
    node.init("test");
    //ros::service::waitForService("/hardware_interface/lightbar/set_lights", ros::Duration(60, 0));
    node.getWorker()->control_priorities.push_back("tester_left");
    node.getWorker()->control_priorities.push_back("tester_right");
    ROS_ERROR_STREAM("Below 'LightBarManager was not able to set light...' errors are expected");
    automotive_platform_msgs::TurnSignalCommandPtr msg_ptr = boost::make_shared<automotive_platform_msgs::TurnSignalCommand>();
    msg_ptr->mode = 1;
    // turn right (no prev owner)
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
    node.turnSignalCallback(msg_ptr);
    std::map<lightbar_manager::LightBarIndicator, std::string> owners = node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    node.turnSignalCallback(msg_ptr);
    owners = node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("") == 0);

    // through other mutually inclusive lights, it was on before the turn
    node.getWorker()->requestControl({YELLOW_ARROW_LEFT}, "tester_left");
    node.getWorker()->requestControl({YELLOW_ARROW_RIGHT}, "tester_unaffected");
    owners =node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_unaffected") == 0);
    // turn left and finish
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
    node.turnSignalCallback(msg_ptr);
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    node.turnSignalCallback(msg_ptr);
    // make sure the previous owner is there
    owners = node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_unaffected") == 0);

    // through other mutually inclusive lights, it was on before the turn
    node.getWorker()->requestControl({YELLOW_ARROW_RIGHT}, "tester_right");
    owners =node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_right") == 0);
    // turn right and don't finish 
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
    node.turnSignalCallback(msg_ptr);
    // middle of turn
    owners =node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    node.turnSignalCallback(msg_ptr);
    owners =node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("lightbar_manager") == 0);
    msg_ptr->turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
    node.turnSignalCallback(msg_ptr);
    // make sure the previous owner is there
    owners = node.getWorker()->getIndicatorControllers();
    EXPECT_TRUE(owners[YELLOW_ARROW_LEFT].compare("tester_left") == 0);
    EXPECT_TRUE(owners[YELLOW_ARROW_RIGHT].compare("tester_right") == 0);

}
} // namespace lightbar_manager

/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lightbar_manager_test");
    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}