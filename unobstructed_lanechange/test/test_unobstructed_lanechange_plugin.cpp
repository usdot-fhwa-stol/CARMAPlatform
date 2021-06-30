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

#include "unobstructed_lanechange.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include "TestHelpers.h"
#include <thread>
#include <chrono>
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/geometry/LineString.h>
#include <string>
#include <cav_msgs/Maneuver.h>
#include <carma_utils/containers/containers.h>
#include <cav_msgs/VehicleState.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_srvs/PlanTrajectory.h>

namespace unobstructed_lanechange
{
    TEST(UnobstructedLaneChangePlugin,Testusingosm){
        // File to process. 
        std::string file = "../resource/map/town01_vector_map_lane_change.osm";
        lanelet::Id start_id = 101;
        lanelet::Id lane_change_start_id = 101;
        lanelet::Id end_id = 107;
        int projector_type = 0;
        std::string target_frame;
        lanelet::ErrorMessages load_errors;
        // Parse geo reference info from the original lanelet map (.osm)
        lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
        lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
        lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
        if (map->laneletLayer.size() == 0)
        {
            FAIL() << "Input map does not contain any lanelets";
        }
        std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //Set Route
        carma_wm::test::setRouteByIds({start_id,end_id},cmw);
        cmw->carma_wm::CARMAWorldModel::setMap(map);
        //get starting position
        auto llt=map.get()->laneletLayer.get(start_id);
        lanelet::LineString3d left_bound=llt.leftBound();
        lanelet::LineString3d right_bound=llt.rightBound();
        geometry_msgs::PoseStamped left;
        geometry_msgs::PoseStamped right;
        for(lanelet::Point3d& p : left_bound)
        {
            left.pose.position.x=p.x();
            left.pose.position.y=p.y();
            left.pose.position.z=p.z();

        }
        for(lanelet::Point3d& p : right_bound)
        {
            right.pose.position.x=p.x();
            right.pose.position.y=p.y();
            right.pose.position.z=p.z();
        }
        geometry_msgs::PoseStamped curr_pos;
        curr_pos.pose.position.x=(left.pose.position.x+right.pose.position.x)/2;
        curr_pos.pose.position.y=(left.pose.position.y+right.pose.position.y)/2;
        curr_pos.pose.position.z=(left.pose.position.z+right.pose.position.z)/2;

        curr_pos.pose.orientation.x=0.0;
        curr_pos.pose.orientation.y=0.0;
        curr_pos.pose.orientation.z=0.0;
        curr_pos.pose.orientation.w=0.0;

        //Define arguments for function maneuvers_to_points
        UnobstructedLaneChangePlugin worker;
        ros::Time::init(); //Initialize ros::Time

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        lanelet::BasicPoint2d veh_pos=shortest_path[0].centerline2d().front();
        double starting_downtrack = cmw->routeTrackPos(veh_pos).downtrack;
        ending_downtrack = cmw->routeTrackPos(shortest_path.back().centerline2d().back()).downtrack;

        worker.wm_ = cmw;

        //Define lane change maneuver
        cav_msgs::Maneuver maneuver;
        maneuver.type=cav_msgs::Maneuver::LANE_CHANGE;
        maneuver.lane_change_maneuver.start_dist = starting_downtrack;
        maneuver.lane_change_maneuver.end_dist = ending_downtrack;
        maneuver.lane_change_maneuver.start_speed = 5.0;
        maneuver.lane_change_maneuver.start_time = ros::Time::now();
        //calculate end_time assuming constant acceleration
        double acc = pow(maneuver.lane_change_maneuver.start_speed,2)/(2*(ending_downtrack - starting_downtrack));
        double end_time = maneuver.lane_change_maneuver.start_speed/acc;
        maneuver.lane_change_maneuver.end_speed = 25.0;
        maneuver.lane_change_maneuver.end_time = ros::Time(end_time + 10.0);
        maneuver.lane_change_maneuver.starting_lane_id = std::to_string(lane_change_start_id);
        maneuver.lane_change_maneuver.ending_lane_id = std::to_string(end_id);
        
        std::vector<cav_msgs::Maneuver> maneuvers;
        maneuvers.push_back(maneuver);
        worker.current_speed_ = maneuver.lane_change_maneuver.start_speed;
        cav_msgs::VehicleState vehicle_state;
        vehicle_state.X_pos_global = veh_pos.x();
        vehicle_state.Y_pos_global = veh_pos.y();
        

        /* Test PlanTrajectory cb */
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        
        // Create a second maneuver of a different type to test the final element in resp.related_maneuvers
        cav_msgs::Maneuver maneuver2;
        maneuver2.type = cav_msgs::Maneuver::LANE_FOLLOWING;

        maneuvers_msg.push_back(maneuver);
        maneuvers_msg.push_back(maneuver2);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        req.maneuver_index_to_plan = 0;
        bool isTrajectory = worker.plan_trajectory_cb(req,resp);
        
        EXPECT_TRUE(isTrajectory);
        EXPECT_TRUE(resp.trajectory_plan.trajectory_points.size() > 2);
        EXPECT_EQ(0, resp.related_maneuvers.back());

    }

    TEST(InLaneCruisingPluginTest, test_verify_yield)
{

  UnobstructedLaneChangePlugin plugin;

  std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;

    ros::Time startTime(ros::Time::now());

    cav_msgs::TrajectoryPlanPoint point_2;
    point_2.x = 5.0;
    point_2.y = 0.0;
    point_2.target_time = startTime + ros::Duration(1);
    point_2.lane_id = "1";
    trajectory_points.push_back(point_2);

    cav_msgs::TrajectoryPlanPoint point_3;
    point_3.x = 10.0;
    point_3.y = 0.0;
    point_3.target_time = startTime + ros::Duration(2);
    point_3.lane_id = "1";
    trajectory_points.push_back(point_3);


    cav_msgs::TrajectoryPlan tp;
    tp.trajectory_id = "yield";
    tp.trajectory_points = trajectory_points;

    std::string traj_id = "yield";
    bool res = plugin.validate_yield_plan(tp, traj_id);
    ASSERT_TRUE(plugin.validate_yield_plan(tp, traj_id));
    std::string traj_id2 = "yieldd";
    ASSERT_FALSE(plugin.validate_yield_plan(tp, traj_id2));

    cav_msgs::TrajectoryPlan tp2;

    cav_msgs::TrajectoryPlanPoint point_4;
    point_4.x = 5.0;
    point_4.y = 0.0;
    point_4.target_time = startTime + ros::Duration(1);
    point_4.lane_id = "1";
    tp2.trajectory_points.push_back(point_4);
    
    ASSERT_FALSE(plugin.validate_yield_plan(tp2, traj_id));

    cav_msgs::TrajectoryPlan tp3;

    cav_msgs::TrajectoryPlanPoint point_5;
    point_5.x = 5.0;
    point_5.y = 0.0;
    point_5.target_time = startTime;
    point_5.lane_id = "1";
    tp3.trajectory_points.push_back(point_5);

    cav_msgs::TrajectoryPlanPoint point_6;
    point_6.x = 10.0;
    point_6.y = 0.0;
    point_6.target_time = startTime + ros::Duration(1);
    point_6.lane_id = "1";
    tp3.trajectory_points.push_back(point_6);

    ASSERT_FALSE(plugin.validate_yield_plan(tp2, traj_id));

    
}


}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    return res;
}