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

#include "test_utils.h"
#include "fixed_priority_cost_function.hpp"

namespace arbitrator
{
    TEST_F(FixedPriorityCostFunctionTest, testSingleManeuver)
    {
        ros::Time::init();
        cav_msgs::ManeuverPlan plan;
        cav_msgs::Maneuver mvr1;
        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_dist = 0;
        mvr1.lane_following_maneuver.start_time = ros::Time::now();
        mvr1.lane_following_maneuver.lane_id.push_back(0);
        mvr1.lane_following_maneuver.end_dist = 1;
        mvr1.lane_following_maneuver.end_time = ros::Time::now();
        mvr1.lane_following_maneuver.start_speed = 10;
        mvr1.lane_following_maneuver.end_speed = 10;
        mvr1.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr1.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";
        plan.maneuvers.push_back(mvr1);
        double cost = fpcf.compute_total_cost(plan);

        ASSERT_NEAR(0.333, cost, 0.01);
    }

    TEST_F(FixedPriorityCostFunctionTest, testMixedPlanners)
    {
        ros::Time::init();
        cav_msgs::ManeuverPlan plan;
        cav_msgs::Maneuver mvr1;
        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_dist = 0;
        mvr1.lane_following_maneuver.start_time = ros::Time::now();
        mvr1.lane_following_maneuver.lane_id.push_back(0);
        mvr1.lane_following_maneuver.end_dist = 1;
        mvr1.lane_following_maneuver.end_time = ros::Time::now();
        mvr1.lane_following_maneuver.start_speed = 10;
        mvr1.lane_following_maneuver.end_speed = 10;
        mvr1.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr1.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";

        cav_msgs::Maneuver mvr2;
        mvr2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_dist = 1;
        mvr2.lane_following_maneuver.start_time = ros::Time::now();
        mvr2.lane_following_maneuver.lane_id.push_back(0);
        mvr2.lane_following_maneuver.end_dist = 2;
        mvr2.lane_following_maneuver.end_time = ros::Time::now();
        mvr2.lane_following_maneuver.start_speed = 10;
        mvr2.lane_following_maneuver.end_speed = 10;
        mvr2.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr2.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_b";
        plan.maneuvers.push_back(mvr1);
        plan.maneuvers.push_back(mvr2);
        double cost = fpcf.compute_total_cost(plan);

        ASSERT_NEAR(0.999, cost, 0.01);
    }

    TEST_F(FixedPriorityCostFunctionTest, testSingleManeuverUnitDistance)
    {
        ros::Time::init();
        cav_msgs::ManeuverPlan plan1;
        cav_msgs::Maneuver mvr1;
        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_dist = 0;
        mvr1.lane_following_maneuver.start_time = ros::Time::now();
        mvr1.lane_following_maneuver.lane_id.push_back(0);
        mvr1.lane_following_maneuver.end_dist = 1;
        mvr1.lane_following_maneuver.end_time = ros::Time::now();
        mvr1.lane_following_maneuver.start_speed = 10;
        mvr1.lane_following_maneuver.end_speed = 10;
        mvr1.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr1.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";
        plan1.maneuvers.push_back(mvr1);
        double cost1 = fpcf.compute_cost_per_unit_distance(plan1);

        ASSERT_NEAR(0.333, cost1, 0.01);
    }

    TEST_F(FixedPriorityCostFunctionTest, testUnitDistanceCost)
    {
        ros::Time::init();
        cav_msgs::ManeuverPlan plan2;
        cav_msgs::Maneuver mvr2;
        mvr2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_dist = 0;
        mvr2.lane_following_maneuver.start_time = ros::Time::now();
        mvr2.lane_following_maneuver.lane_id.push_back(0);
        mvr2.lane_following_maneuver.end_dist = 1;
        mvr2.lane_following_maneuver.end_time = ros::Time::now();
        mvr2.lane_following_maneuver.start_speed = 10;
        mvr2.lane_following_maneuver.end_speed = 10;
        mvr2.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr2.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_a";
        cav_msgs::Maneuver mvr3;
        mvr3.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr3.lane_following_maneuver.start_dist = 1;
        mvr3.lane_following_maneuver.start_time = ros::Time::now();
        mvr3.lane_following_maneuver.lane_id.push_back(0);
        mvr3.lane_following_maneuver.end_dist = 2;
        mvr3.lane_following_maneuver.end_time = ros::Time::now();
        mvr3.lane_following_maneuver.start_speed = 10;
        mvr3.lane_following_maneuver.end_speed = 10;
        mvr3.lane_following_maneuver.parameters.maneuver_id.push_back(0);
        mvr3.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_c";
        plan2.maneuvers.push_back(mvr2);
        plan2.maneuvers.push_back(mvr3);
        double cost2 = fpcf.compute_cost_per_unit_distance(plan2);
        ASSERT_NEAR(0.333/2.0, cost2, 0.01);
    }
}