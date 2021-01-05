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
#include "tree_planner.hpp"
#include <gmock/gmock.h>

using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnArg;
using ::testing::InSequence;

namespace arbitrator
{

    class MockSearchStrategy : public SearchStrategy
    {
        public:
            using PlanAndCost = std::pair<cav_msgs::ManeuverPlan, double>;
            MOCK_CONST_METHOD1(prioritize_plans, std::vector<PlanAndCost>(std::vector<PlanAndCost>));
            ~MockSearchStrategy(){};
    };

    class MockCostFunction : public CostFunction
    {
        public:
            MOCK_METHOD1(compute_total_cost, double(const cav_msgs::ManeuverPlan&));
            MOCK_METHOD1(compute_cost_per_unit_distance, double(const cav_msgs::ManeuverPlan&));
            ~MockCostFunction(){};

    };

    class MockNeighborGenerator : public NeighborGenerator
    {
        public:
            MOCK_CONST_METHOD1(generate_neighbors, std::vector<cav_msgs::ManeuverPlan>(cav_msgs::ManeuverPlan));
            ~MockNeighborGenerator(){};
    };

    class TreePlannerTest : public ::testing::Test 
    {
        public:
            TreePlannerTest():
                tp{mcf, mng, mss, ros::Duration(5)} {};
            MockSearchStrategy mss;
            MockCostFunction mcf;
            MockNeighborGenerator mng;
            TreePlanner tp;
    };

    TEST_F(TreePlannerTest, testGeneratePlan1)
    {
        cav_msgs::ManeuverPlan plan1;
        cav_msgs::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = ros::Time(0);
        mvr1.lane_following_maneuver.end_time = ros::Time(5.0);

        plan1.maneuvers.push_back(mvr1);
        std::vector<cav_msgs::ManeuverPlan> plans{plan1};

        {
            InSequence seq;
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillOnce(
                    Return(plans)
                );
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillRepeatedly(
                    Return(std::vector<cav_msgs::ManeuverPlan>())
                );
        }

        EXPECT_CALL(mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        cav_msgs::ManeuverPlan plan = tp.generate_plan();
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(1, plan.maneuvers.size());
        ASSERT_EQ(ros::Time(0), plan.maneuvers[0].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(5), plan.maneuvers[0].lane_following_maneuver.end_time);
    }

    TEST_F(TreePlannerTest, testGeneratePlan2)
    {
        cav_msgs::ManeuverPlan plan1, plan2, plan3;
        cav_msgs::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = ros::Time(0);
        mvr1.lane_following_maneuver.end_time = ros::Time(5.0);

        mvr2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_time = ros::Time(0);
        mvr2.lane_following_maneuver.end_time = ros::Time(5.0);

        mvr3.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr3.lane_following_maneuver.start_time = ros::Time(0);
        mvr3.lane_following_maneuver.end_time = ros::Time(5.0);

        plan1.maneuvers.push_back(mvr1);
        plan2.maneuvers.push_back(mvr2);
        plan3.maneuvers.push_back(mvr3);
        std::vector<cav_msgs::ManeuverPlan> plans{plan1, plan2, plan3};

        {
            InSequence seq;
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillOnce(
                    Return(plans)
                );
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillRepeatedly(
                    Return(std::vector<cav_msgs::ManeuverPlan>())
                );
        }

        EXPECT_CALL(mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        cav_msgs::ManeuverPlan plan = tp.generate_plan();
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(1, plan.maneuvers.size());
        ASSERT_EQ(ros::Time(0), plan.maneuvers[0].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(5), plan.maneuvers[0].lane_following_maneuver.end_time);
    }

    TEST_F(TreePlannerTest, testGeneratePlan3)
    {
        cav_msgs::ManeuverPlan plan1, plan2, plan3;
        cav_msgs::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = ros::Time(0);
        mvr1.lane_following_maneuver.end_time = ros::Time(2);

        mvr2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_time = ros::Time(2);
        mvr2.lane_following_maneuver.end_time = ros::Time(4);

        mvr3.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        mvr3.lane_following_maneuver.start_time = ros::Time(4);
        mvr3.lane_following_maneuver.end_time = ros::Time(5);

        plan1.maneuvers.push_back(mvr1);

        plan2.maneuvers.push_back(mvr1);
        plan2.maneuvers.push_back(mvr2);

        plan3.maneuvers.push_back(mvr1);
        plan3.maneuvers.push_back(mvr2);
        plan3.maneuvers.push_back(mvr3);

        std::vector<cav_msgs::ManeuverPlan> plans1{plan1};
        std::vector<cav_msgs::ManeuverPlan> plans2{plan2};
        std::vector<cav_msgs::ManeuverPlan> plans3{plan3};

        {
            InSequence seq;
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillOnce(
                    Return(plans1)
                );
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillOnce(
                    Return(plans2)
                );
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillOnce(
                    Return(plans3)
                );
            EXPECT_CALL(mng, generate_neighbors(_))
                .WillRepeatedly(
                    Return(std::vector<cav_msgs::ManeuverPlan>())
                );
        }

        EXPECT_CALL(mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        cav_msgs::ManeuverPlan plan = tp.generate_plan();
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(3, plan.maneuvers.size());
        ASSERT_EQ(ros::Time(0), plan.maneuvers[0].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(2), plan.maneuvers[0].lane_following_maneuver.end_time);
        ASSERT_EQ(ros::Time(2), plan.maneuvers[1].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(4), plan.maneuvers[1].lane_following_maneuver.end_time);
        ASSERT_EQ(ros::Time(4), plan.maneuvers[2].lane_following_maneuver.start_time);
        ASSERT_EQ(ros::Time(5), plan.maneuvers[2].lane_following_maneuver.end_time);
    }
}