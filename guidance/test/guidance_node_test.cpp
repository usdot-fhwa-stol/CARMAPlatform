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
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <cav_msgs/GuidanceState.h>
#include <thread>
#include <chrono>

TEST(GuidanceNodeTest, TestGuidanceLaunch) {
    ros::NodeHandle nh = ros::NodeHandle();
    const cav_msgs::GuidanceState* state;
    ros::Subscriber state_sub = nh.subscribe<cav_msgs::GuidanceState>("state", 10, [&](cav_msgs::GuidanceStateConstPtr msg){
        state = msg.get();
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_EQ(cav_msgs::GuidanceState::STARTUP, state->state);
}


/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "guidance_test");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
