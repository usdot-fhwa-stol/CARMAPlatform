/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "mpc_follower_wrapper/mpc_follower_wrapper_worker.hpp"

#include <gtest/gtest.h>
#include <iostream>


TEST(TrajectoryPlanPointToWaypointConverterTest, test1)
{  

    cav_msgs::TrajectoryPlanPoint tpp;
    cav_msgs::TrajectoryPlanPoint tpp2;
    mpc_follower_wrapper::MPCFollowerWrapperWorker mpcww;
    autoware_msgs::Waypoint waypoint;

    tpp.x = 10;
    tpp.y = 10;
    tpp.target_time = ros::Time(0, 10);

    tpp2.x = 12;
    tpp2.y = 12;
    tpp2.target_time = ros::Time(0, 20);

    waypoint = mpcww.TrajectoryPlanPointToWaypointConverter(tpp, tpp2);

    double v_x = waypoint.twist.twist.linear.x;
    double correct_v_x = 28.28;

    EXPECT_NEAR(correct_v_x, v_x, 0.01);

    tpp.x = 0;
    tpp.y = 0;
    tpp.target_time = ros::Time(0);

    tpp2.x = 1;
    tpp2.y = -1;
    tpp2.target_time = ros::Time(0, 10);

    waypoint = mpcww.TrajectoryPlanPointToWaypointConverter(tpp, tpp2);

    v_x = waypoint.twist.twist.linear.x;
    correct_v_x = 14.14;

    EXPECT_NEAR(correct_v_x, v_x, 0.01);

}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
