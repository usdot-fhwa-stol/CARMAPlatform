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

#include <yield_plugin/yield_plugin.h>
#include <yield_plugin/yield_plugin_node.h>
#include <carma_utils/CARMAUtils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>


TEST(YieldPlugin, UnitTestYield)
{
    
    cav_msgs::TrajectoryPlan original_tp;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;
    cav_msgs::TrajectoryPlanPoint trajectory_point_6;
    cav_msgs::TrajectoryPlanPoint trajectory_point_7;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.target_time = ros::Time(0);

    trajectory_point_2.x = 5.0;
    trajectory_point_2.y = 1.0;
    trajectory_point_2.target_time = ros::Time(1);

    trajectory_point_3.x = 10.0;
    trajectory_point_3.y = 1.0;
    trajectory_point_3.target_time = ros::Time(2);
    
    trajectory_point_4.x = 15.0;
    trajectory_point_4.y = 1.0;
    trajectory_point_4.target_time = ros::Time(3);

    trajectory_point_5.x = 20.0;
    trajectory_point_5.y = 1.0;
    trajectory_point_5.target_time = ros::Time(4);

    trajectory_point_6.x = 25.0;
    trajectory_point_6.y = 1.0;
    trajectory_point_6.target_time = ros::Time(5);

    trajectory_point_7.x = 30.0;
    trajectory_point_7.y = 1.0;
    trajectory_point_7.target_time = ros::Time(6);
    
    original_tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5, trajectory_point_6, trajectory_point_7};
    
    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver maneuver;
    maneuver.type = maneuver.LANE_FOLLOWING;
    maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "InLane_Cruising";
    plan.maneuvers.push_back(maneuver);

    cav_srvs::PlanTrajectory traj_srv;
    traj_srv.request.initial_trajectory_plan = original_tp;
    traj_srv.request.vehicle_state.X_pos_global= 1;
    traj_srv.request.vehicle_state.Y_pos_global = 1;
    traj_srv.request.vehicle_state.longitudinal_vel = 11;
    traj_srv.request.maneuver_plan = plan;
    
    

    double res = 0;

    ros::CARMANodeHandle nh;
    ros::ServiceClient plugin1= nh.serviceClient<cav_srvs::PlanTrajectory>("plugins/Yieldlugin/plan_trajectory");

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    if (plugin1.call(traj_srv))
    {   
        ROS_INFO("Service called");
        res = traj_srv.response.trajectory_plan.trajectory_points.size();
    }
    else
    {
        ROS_INFO("Failed to call service");
        res=0;
    }

    EXPECT_EQ(original_tp.trajectory_points.size(), res);
}


int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_yield_plugin");
    auto res = RUN_ALL_TESTS();
    return res;
}