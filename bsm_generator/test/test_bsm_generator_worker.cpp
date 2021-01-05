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

#include "bsm_generator_worker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(BSMWorkerTest, testMsgCount)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_EQ(0, worker.getNextMsgCount());
    for(int i = 1; i < 127; i++)
    {
        worker.getNextMsgCount();
    }
    EXPECT_EQ(127, worker.getNextMsgCount());
    EXPECT_EQ(0, worker.getNextMsgCount());
}

TEST(BSMWorkerTest, testMsgId)
{
    bsm_generator::BSMGeneratorWorker worker;
    ros::Time time1(10, 0);
    std::vector<uint8_t> msgId1 = worker.getMsgId(time1);
    ros::Time time2(11, 0);
    std::vector<uint8_t> msgId2 = worker.getMsgId(time2);
    ros::Time time3(310, 0);
    std::vector<uint8_t> msgId3 = worker.getMsgId(time3);
    EXPECT_TRUE(msgId1 == msgId2);
    EXPECT_TRUE(msgId2 != msgId3);
}

TEST(BSMWorkerTest, testSecMark)
{
    bsm_generator::BSMGeneratorWorker worker;
    ros::Time time1(10, 0);
    EXPECT_EQ(10000, worker.getSecMark(time1));
    ros::Time time2(70, 0);
    EXPECT_EQ(10000, worker.getSecMark(time2));
    ros::Time time3(71, 0);
    EXPECT_EQ(11000, worker.getSecMark(time3));
}

TEST(BSMWorkerTest, testSpeedInRange)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_NEAR(163.8, worker.getSpeedInRange(170.0f), 0.01);
    EXPECT_NEAR(0, worker.getSpeedInRange(-1.0f), 0.01);
    EXPECT_NEAR(15.3, worker.getSpeedInRange(15.3f), 0.01);
}

TEST(BSMWorkerTest, testSteerWheelAngleInRnage)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_NEAR(189.0, worker.getSteerWheelAngleInRnage(3.316), 0.01);
    EXPECT_NEAR(-189.0, worker.getSteerWheelAngleInRnage(-3.316), 0.01);
    EXPECT_NEAR(57.2958, worker.getSteerWheelAngleInRnage(1), 0.01);
}

TEST(BSMWorkerTest, testLongAccelInRange)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_NEAR(20.0, worker.getLongAccelInRange(21.0f), 0.01);
    EXPECT_NEAR(-20.0, worker.getLongAccelInRange(-21.0f), 0.01);
    EXPECT_NEAR(2.5, worker.getLongAccelInRange(2.5f), 0.01);
}

TEST(BSMWorkerTest, testYawRateInRange)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_NEAR(327.67, worker.getYawRateInRange(330.0f), 0.01);
    EXPECT_NEAR(-327.67, worker.getYawRateInRange(-330.0f), 0.01);
    EXPECT_NEAR(9.1, worker.getYawRateInRange(9.1f), 0.01);
}

TEST(BSMWorkerTest, testBrakeAppliedStatus)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_EQ(0b1111, worker.getBrakeAppliedStatus(0.5));
    EXPECT_EQ(0, worker.getBrakeAppliedStatus(0.049));
    EXPECT_EQ(0, worker.getBrakeAppliedStatus(0));
}

TEST(BSMWorkerTest, testHeading)
{
    bsm_generator::BSMGeneratorWorker worker;
    EXPECT_NEAR(359.9875f, worker.getHeadingInRange(360.0f), 0.0001);
    EXPECT_NEAR(0.0f, worker.getHeadingInRange(-60.0f), 0.0001);
    EXPECT_NEAR(300.001f, worker.getHeadingInRange(300.001f), 0.0001);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
