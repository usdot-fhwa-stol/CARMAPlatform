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

#include <gtest/gtest.h>
#include "cost_plugin_worker.hpp"

namespace cost_plugin_system
{
TEST(CostPluginWorkerTest, testSingleManeuver)
{
    cost_plugin_system::CostPluginWorker cpw;
    ros::Time::init();
    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr1;
    mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
    mvr1.lane_following_maneuver.start_dist = 0;
    mvr1.lane_following_maneuver.start_time = ros::Time(0);
    mvr1.lane_following_maneuver.lane_id.push_back(0);
    mvr1.lane_following_maneuver.end_dist = 1;
    mvr1.lane_following_maneuver.end_time = ros::Time(1.0);
    mvr1.lane_following_maneuver.start_speed = 10;
    mvr1.lane_following_maneuver.end_speed = 10;
    mvr1.lane_following_maneuver.parameters.maneuver_id.push_back(0);
    mvr1.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";
    plan.maneuvers.push_back(mvr1);
    double cost = cpw.compute_final_score(plan);

    ASSERT_NEAR(0.824, cost, 0.01);
}

TEST(CostPluginWorkerTest, testMixedPlanners)
{
    cost_plugin_system::CostPluginWorker cpw;
    ros::Time::init();
    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr1;
    mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
    mvr1.lane_following_maneuver.start_dist = 0;
    mvr1.lane_following_maneuver.start_time = ros::Time(0);
    mvr1.lane_following_maneuver.lane_id.push_back(0);
    mvr1.lane_following_maneuver.end_dist = 1;
    mvr1.lane_following_maneuver.end_time = ros::Time(1.0);
    mvr1.lane_following_maneuver.start_speed = 10;
    mvr1.lane_following_maneuver.end_speed = 10;
    mvr1.lane_following_maneuver.parameters.maneuver_id.push_back(0);
    mvr1.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";

    cav_msgs::Maneuver mvr2;
    mvr2.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr2.lane_following_maneuver.start_dist = 1;
    mvr2.lane_following_maneuver.start_time = ros::Time(1.0);
    mvr2.lane_following_maneuver.lane_id.push_back(0);
    mvr2.lane_following_maneuver.end_dist = 5;
    mvr2.lane_following_maneuver.end_time = ros::Time(5.0);
    mvr2.lane_following_maneuver.start_speed = 10;
    mvr2.lane_following_maneuver.end_speed = 0;
    mvr2.lane_following_maneuver.parameters.maneuver_id.push_back(0);
    mvr2.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_b";
    plan.maneuvers.push_back(mvr1);
    plan.maneuvers.push_back(mvr2);
    double cost = cpw.compute_final_score(plan);

    ASSERT_NEAR(0.981, cost, 0.01);
}
} // namespace cost_plugin_system