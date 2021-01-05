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
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanManeuvers.h>
#include <gmock/gmock.h>
#include "plugin_neighbor_generator.hpp"

namespace arbitrator
{
    class MockCapabilitiesInterface 
    {
        public:
            using PluginResponses = std::map<std::string, cav_srvs::PlanManeuvers>;
            MOCK_METHOD2(get_plans, PluginResponses(std::string, cav_srvs::PlanManeuvers));
            MOCK_METHOD1(get_topics_for_capability, std::vector<std::string>(const std::string&));

            template<typename MSrv>
            std::map<std::string, MSrv> multiplex_service_call_for_capability(std::string query_string, MSrv msg);
            ~MockCapabilitiesInterface(){};

    };

    template<>
    std::map<std::string, cav_srvs::PlanManeuvers> 
    MockCapabilitiesInterface::multiplex_service_call_for_capability(
        std::string query_string, 
        cav_srvs::PlanManeuvers msg)
    {
        return get_plans(query_string, msg);
    }

    class PluginNeighborGeneratorTest : public ::testing::Test 
    {
        public:
            PluginNeighborGeneratorTest():
             png{mci} {};

            MockCapabilitiesInterface mci;
            PluginNeighborGenerator<MockCapabilitiesInterface> png;
    };

    TEST_F(PluginNeighborGeneratorTest, testInitalize)
    {
        // Verify that initialize just doesn't throw any exceptions
    }

    TEST_F(PluginNeighborGeneratorTest, testGetNeighbors1)
    {
        EXPECT_CALL(mci, 
            get_plans(::testing::_, ::testing::_))
            .WillRepeatedly(
                ::testing::Return(
                    std::map<std::string, cav_srvs::PlanManeuvers>()));

        cav_msgs::ManeuverPlan plan;
        std::vector<cav_msgs::ManeuverPlan> plans = png.generate_neighbors(plan);

        ASSERT_EQ(0, plans.size());
        ASSERT_TRUE(plans.empty());
    }

    TEST_F(PluginNeighborGeneratorTest, testGetNeighbors2)
    {
        std::map<std::string, cav_srvs::PlanManeuvers> responses;
        cav_srvs::PlanManeuvers resp1, resp2, resp3;
        resp1.response.new_plan.maneuver_plan_id.push_back(0);
        resp2.response.new_plan.maneuver_plan_id.push_back(1);
        resp3.response.new_plan.maneuver_plan_id.push_back(2);
        responses["test_plugin_a"] = resp1;
        responses["test_plugin_b"] = resp2;
        responses["test_plugin_c"] = resp3;

        EXPECT_CALL(mci, 
            get_plans(::testing::_, ::testing::_))
            .WillRepeatedly(
                ::testing::Return(
                    responses));

        cav_msgs::ManeuverPlan plan;
        std::vector<cav_msgs::ManeuverPlan> plans = png.generate_neighbors(plan);

        ASSERT_FALSE(plans.empty());
        ASSERT_EQ(3, plans.size());
        ASSERT_EQ(0, plans[0].maneuver_plan_id[0]);
        ASSERT_EQ(1, plans[1].maneuver_plan_id[0]);
        ASSERT_EQ(2, plans[2].maneuver_plan_id[0]);
    }
}