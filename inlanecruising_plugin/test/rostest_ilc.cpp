/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <inlanecruising_plugin/inlanecruising_plugin_node.h>
#include <carma_utils/CARMAUtils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "ros/service.h"
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include <cav_srvs/PlanTrajectory.h>

TEST(InLaneCruisingPluginTest, rostest1)
{
    

    ros::CARMANodeHandle nh;
    bool flag_trajectory = false;
    bool flag_yield = false;
    std::string res = "";

    cav_srvs::PlanTrajectory traj_srv;
    traj_srv.request.initial_trajectory_plan.trajectory_id = "ILCReq";
   
    ros::ServiceClient plugin1= nh.serviceClient<cav_srvs::PlanTrajectory>("plugins/InLaneCruisingPlugin/plan_trajectory");

    ROS_INFO_STREAM("ilc service: " << plugin1.getService());
    if(plugin1.waitForExistence(ros::Duration(5.0)))
    {
        ros::spinOnce();
        ROS_ERROR("waiting");
        if (plugin1.call(traj_srv))
        {
            res = traj_srv.response.trajectory_plan.trajectory_id;
            ROS_ERROR("ILC Traj Service called");
            flag_trajectory = true;
            flag_yield = true;
            
        }
        else
        {
            ROS_ERROR("ILC Trajectory Service not called");
            res = "error";
        }
    }
    EXPECT_TRUE(flag_trajectory);
    ASSERT_TRUE(flag_yield);
    EXPECT_EQ(res, "ILC2Yield");
}



int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rostest_ilc");
    ros::NodeHandle nh;
    auto res = RUN_ALL_TESTS();
    return res;
}

