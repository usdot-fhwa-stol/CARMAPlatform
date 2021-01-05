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

#include "plugin_manager.h"
#include <gtest/gtest.h>

namespace health_monitor
{    
    TEST(PluginManagerTest, testRegisteredPlugins)
    {
        std::vector<std::string> required_plugins{"autoware", "pure_pursuit"};
        PluginManager pm(required_plugins, "/guidance/plugin/", "/plan_maneuver", "/plan_trajectory");
        cav_msgs::Plugin msg1;
        msg1.name = "autoware";
        msg1.available = true;
        msg1.activated = false;
        msg1.type = cav_msgs::Plugin::STRATEGIC;
        msg1.versionId = "1.0.1";
        msg1.capability = "waypoint_following/autoware";
        cav_msgs::PluginConstPtr msg1_pointer(new cav_msgs::Plugin(msg1));
        pm.update_plugin_status(msg1_pointer);
        cav_msgs::Plugin msg2;
        msg2.name = "lane_change";
        msg2.available = true;
        msg2.activated = true;
        // this field is not used by plugin manager
        msg2.type = cav_msgs::Plugin::TACTICAL;
        msg2.versionId = "1.0.0";
        msg2.capability = "lane_change";
        cav_msgs::PluginConstPtr msg2_pointer(new cav_msgs::Plugin(msg2));
        pm.update_plugin_status(msg2_pointer);
        cav_srvs::PluginListResponse res;
        pm.get_registered_plugins(res);
        EXPECT_EQ(2, res.plugins.size());
        // for cruising plugin
        EXPECT_EQ(true, res.plugins.begin()->activated);
        EXPECT_EQ(true, res.plugins.begin()->available);
        EXPECT_EQ(0, res.plugins.begin()->name.compare("autoware"));
        //EXPECT_EQ(0, res.plugins.begin()->type);
        //EXPECT_EQ(0, res.plugins.begin()->versionId.compare(""));
        // for lane change plugin
        EXPECT_EQ(false, std::prev(res.plugins.end())->activated);
        EXPECT_EQ(true, std::prev(res.plugins.end())->available);
        EXPECT_EQ(0, std::prev(res.plugins.end())->name.compare("lane_change"));
        //EXPECT_EQ(0, std::prev(res.plugins.end())->type);
        //EXPECT_EQ(0, std::prev(res.plugins.end())->versionId.compare(""));
        // TODO need to handle type and verionID
        cav_srvs::PluginListResponse res2;
        pm.get_active_plugins(res2);
        EXPECT_EQ(true, res2.plugins.begin()->activated);
        EXPECT_EQ(true, res2.plugins.begin()->available);
        EXPECT_EQ(0, res2.plugins.begin()->name.compare("autoware"));
        pm.activate_plugin("lane_change", true);
        cav_srvs::PluginListResponse res3;
        pm.get_active_plugins(res3);
        EXPECT_EQ(2, res3.plugins.size());
        EXPECT_EQ(true, std::prev(res3.plugins.end())->activated);
        EXPECT_EQ(true, std::prev(res3.plugins.end())->available);
        EXPECT_EQ(0, std::prev(res3.plugins.end())->name.compare("lane_change"));
        pm.activate_plugin("lane_change", false);
        cav_srvs::PluginListResponse res4;
        pm.get_active_plugins(res4);
        EXPECT_EQ(1, res4.plugins.size());
        EXPECT_EQ(true, res4.plugins.begin()->activated);
        EXPECT_EQ(true, res4.plugins.begin()->available);
        EXPECT_EQ(0, res4.plugins.begin()->name.compare("autoware"));
        cav_srvs::GetPluginApiRequest req;
        req.capability = "";
        cav_srvs::GetPluginApiResponse resp;
        EXPECT_TRUE(pm.get_strategic_plugins_by_capability(req, resp));
        EXPECT_EQ(1, resp.plan_service.size());
        EXPECT_EQ("/guidance/plugin/autoware/plan_maneuver", resp.plan_service[0]);
        resp = cav_srvs::GetPluginApiResponse();
        EXPECT_TRUE(pm.get_tactical_plugins_by_capability(req, resp));
        EXPECT_EQ(1, resp.plan_service.size());
        EXPECT_EQ("/guidance/plugin/lane_change/plan_trajectory", resp.plan_service[0]);
        resp = cav_srvs::GetPluginApiResponse();
        req.capability = "waypoint_following";
        EXPECT_TRUE(pm.get_strategic_plugins_by_capability(req, resp));
        EXPECT_EQ(1, resp.plan_service.size());
        EXPECT_EQ("/guidance/plugin/autoware/plan_maneuver", resp.plan_service[0]);
        resp = cav_srvs::GetPluginApiResponse();
        req.capability = "platooning";
        EXPECT_TRUE(pm.get_strategic_plugins_by_capability(req, resp));
        EXPECT_EQ(0, resp.plan_service.size());
    }

}
