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


#include "localizer.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
namespace localizer{

TEST(GnssNdtSelectorTest, testReliabilityCounter)
{
    //default 
    localizer::NDTReliabilityCounter counter_default;
    EXPECT_EQ(0, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(1.5);
    EXPECT_EQ(0, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(3);
    EXPECT_EQ(1, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(-1);
    EXPECT_EQ(2, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(3);
    counter_default.onNDTScore(3);
    counter_default.onNDTScore(3);
    counter_default.onNDTScore(3);
    EXPECT_EQ(6, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(100);
    EXPECT_EQ(7, counter_default.getNDTReliabilityCounter());
    counter_default.onNDTScore(100);
    EXPECT_EQ(7, counter_default.getNDTReliabilityCounter());
    // other case
    localizer::NDTReliabilityCounter counter(2.0, 2);
    EXPECT_EQ(0, counter.getNDTReliabilityCounter());
    counter.onNDTScore(1.5);
    EXPECT_EQ(0, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(1, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(2, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(3, counter.getNDTReliabilityCounter());
    counter.onNDTScore(1.5);
    EXPECT_EQ(2, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(3, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(4, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(5, counter.getNDTReliabilityCounter());
    counter.onNDTScore(100.0);
    EXPECT_EQ(5, counter.getNDTReliabilityCounter());
    counter.onNDTScore(1.5);
    EXPECT_EQ(4, counter.getNDTReliabilityCounter());
    counter.onNDTScore(1.5);
    counter.onNDTScore(1.5);
    counter.onNDTScore(1.5);
    counter.onNDTScore(1.5);
    EXPECT_EQ(0, counter.getNDTReliabilityCounter());
    counter.onNDTScore(0.1);
    EXPECT_EQ(0, counter.getNDTReliabilityCounter());
}

}


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
