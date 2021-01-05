
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

#include "pure_pursuit.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PurePursuitTest, test1)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = ros::Time(1.0);
    platoon_control::PurePursuit pp;
    double res = pp.calculateSteer(point);
    EXPECT_EQ(0, res);

    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = 4.0;
    point2.y = 4.0;
    point2.target_time = ros::Time(2.0);
    double res2 = pp.calculateSteer(point2);
    EXPECT_NEAR(0.5, res2, 0.1);

}


TEST(PurePursuitTest, test2)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = ros::Time(1.0);
    platoon_control::PurePursuit pp;
    double res = pp.calculateSteer(point);
    EXPECT_EQ(0, res);

    cav_msgs::TrajectoryPlanPoint point2;
    point2.x = -4.0;
    point2.y = -4.0;
    point2.target_time = ros::Time(1.0);
    double res2 = pp.calculateSteer(point2);
    EXPECT_NEAR(-0.5, res2, 0.1);

}
