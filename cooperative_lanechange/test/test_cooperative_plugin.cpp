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

#include "cooperative_lanechange.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
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
#include <cav_msgs/MobilityResponse.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <cav_msgs/LocationECEF.h>
#include <cav_msgs/Trajectory.h>
#include <sstream>
#include <ros/package.h>
#include <cav_msgs/LaneChangeStatus.h>
#include <cav_msgs/PlanType.h>

    TEST(CooperativeLaneChangePlugin,TestTrajectorytoecef)
    {
        cooperative_lanechange::CooperativeLaneChangePlugin worker;

        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_plan;

        std::string base_proj = "+proj=tmerc +lat_0=0.0 +lon_0=0.0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m "
                          "+no_defs";
        std_msgs::String msg;
        msg.data = base_proj;
        std_msgs::StringConstPtr msg_ptr(new std_msgs::String(msg));
        worker.georeference_callback(msg_ptr);  // Set projection

        cav_msgs::TrajectoryPlanPoint point_1;
        point_1.x = 20.0;
        point_1.y = 0.0;
        trajectory_plan.push_back(point_1);
        cav_msgs::LocationECEF ecef_point_1 = worker.trajectory_point_to_ecef(point_1);

        ASSERT_NEAR(ecef_point_1.ecef_x, 637813699.0, 0.001);
        ASSERT_NEAR(ecef_point_1.ecef_y, 1999.0, 0.001);
        ASSERT_NEAR(ecef_point_1.ecef_z, 0.0, 0.001);
        

        cav_msgs::TrajectoryPlanPoint point_2;
        point_2.x = 19.0;
        point_2.y = 0.0;
        trajectory_plan.push_back(point_2);
        ecef_point_1 = worker.trajectory_point_to_ecef(point_2);

        cav_msgs::Trajectory traj = worker.trajectory_plan_to_trajectory(trajectory_plan);
        ASSERT_NEAR(traj.location.ecef_x, 637813699.0, 0.001);
        ASSERT_NEAR(traj.location.ecef_y, 1999.0, 0.001);
        ASSERT_NEAR(traj.location.ecef_z, 0.0, 0.001);

        ASSERT_EQ(traj.offsets.size(), 1);
        ASSERT_NEAR(traj.offsets[0].offset_x, 0.0, 0.001);
        ASSERT_NEAR(traj.offsets[0].offset_y, -100.0, 0.001);
        ASSERT_NEAR(traj.offsets[0].offset_z, 0.0, 0.001);


    }

    TEST(CooperativeLaneChangePlugin,TestLaneChangefunctions)
    {
         // File to process. 
        std::string path = ros::package::getPath("unobstructed_lanechange");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);
        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 101;
        lanelet::Id end_id = 111;
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
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        //Define arguments for function maneuvers_to_points
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
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
        auto points_and_target_speeds = worker.maneuvers_to_points(maneuvers, starting_downtrack, cmw, vehicle_state);  
        

        /* Test plan lanechange */
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_plan = worker.plan_lanechange(req);
        EXPECT_TRUE(traj_plan.size() > 2);

        cav_msgs::MobilityRequest request = worker.create_mobility_request(traj_plan, maneuver);
        EXPECT_EQ(cav_msgs::PlanType::CHANGE_LANE_LEFT, request.plan_type.type);
        /*Test compose trajectort and helper function*/
        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory;

        int nearest_pt = basic_autonomy::waypoint_generation::get_nearest_index_by_downtrack(points_and_target_speeds,cmw,vehicle_state);

        std::vector<lanelet::BasicPoint2d> points_split;
        std::vector<double> speeds_split;
        basic_autonomy::waypoint_generation::split_point_speed_pairs(points_and_target_speeds, &points_split, &speeds_split);
        EXPECT_TRUE(points_split.size() == speeds_split.size());

        //Test trajectory from points
        std::vector<double> yaw_values ={};
        yaw_values.resize(points_and_target_speeds.size(),0);
        std::vector<double> times={};
        times.resize(points_and_target_speeds.size(),0.1); // Sample time vector with all 0.1 speeds
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points = worker.trajectory_from_points_times_orientations(points_split,times,yaw_values, ros::Time::now());

        //Test apply speed limits
        std::vector<double> speed_limits={};
        speed_limits.resize(speeds_split.size(),5);
        std::vector<double> constrained_speeds = worker.apply_speed_limits(speeds_split,speed_limits);

        // Test adaptive lookahead
        double lookahead = worker.get_adaptive_lookahead(5);   
        std::vector<double> lookahead_speeds = worker.get_lookahead_speed(points_split,constrained_speeds, lookahead);

        trajectory = worker.compose_trajectory_from_centerline(points_and_target_speeds, vehicle_state, ros::Time::now(),lane_change_start_id, 15);
        //Valid Trajectory has at least 2 points
        EXPECT_TRUE(trajectory.size() > 2);

        auto route_geometry = worker.create_route_geom(starting_downtrack,start_id, ending_downtrack,cmw);
        lanelet::BasicPoint2d state_pos(vehicle_state.X_pos_global, vehicle_state.Y_pos_global);
        double current_downtrack = cmw->routeTrackPos(state_pos).downtrack;
        int nearest_pt_geom = basic_autonomy::waypoint_generation::get_nearest_index_by_downtrack(route_geometry, cmw ,current_downtrack);

        //Test create lanechange route
        lanelet::Lanelet start_lanelet = map->laneletLayer.get(start_id);
        lanelet::Lanelet end_lanelet = map->laneletLayer.get(end_id);
        lanelet::BasicPoint2d start_position(vehicle_state.X_pos_global, vehicle_state.Y_pos_global);
        lanelet::BasicPoint2d end_position = end_lanelet.centerline2d().basicLineString().back() ;
        lanelet::BasicLineString2d lc_route = worker.create_lanechange_path(start_lanelet, end_lanelet);

        //Test Compute heading frame between two points
        Eigen::Isometry2d frame = worker.compute_heading_frame(start_position, end_position);

    }

    TEST(CooperativeLaneChangePlugin,TestAddManeuvertoResponse){
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;

        req.maneuver_index_to_plan = 0;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        
        lanelet::BasicPoint2d veh_pos(1.0,1.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = 10.0;

        std::vector<cav_msgs::TrajectoryPlanPoint> planned_trajectory ={};
        worker.add_maneuver_to_response(req,resp,planned_trajectory);
        EXPECT_EQ(10.0,resp.trajectory_plan.initial_longitudinal_velocity);
        EXPECT_EQ(0, resp.related_maneuvers.back());
    }

    TEST(CooperativeLaneChangePlugin,Testcurrentgapcb){
        // File to process. 
        std::string path = ros::package::getPath("unobstructed_lanechange");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);
        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 101;
        lanelet::Id end_id = 111;
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
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        //Define arguments for function maneuvers_to_points
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
        ros::Time::init(); //Initialize ros::Time

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        lanelet::BasicPoint2d veh_pos=map.get()->laneletLayer.get(lane_change_start_id).centerline2d().front();
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
        
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
          // //Add roadway object - lag vehicle
        cav_msgs::ExternalObject object;

        // //Test plan trajectory cb
        object.id =1;
        object.object_type = cav_msgs::ExternalObject::SMALL_VEHICLE;
        object.pose.pose.position.x =0.0;
        object.pose.pose.position.y =0.0;
        object.pose.pose.position.z =0.0;
        object.velocity.twist.linear.x = 5.0;
        
        geometry_msgs::Vector3 size;
        size.x = 4;
        size.y = 2;
        size.z = 1;
        
        object.size = size;

        cav_msgs::PredictedState pred;
        auto pred_pose = object.pose.pose;
        pred_pose.position.y += 1;
        pred.predicted_position = pred_pose;
        pred.predicted_position_confidence = 1.0;
        
        object.predictions.push_back(pred);

        cav_msgs::RoadwayObstacle obstacle;
        obstacle.object = object;
        obstacle.connected_vehicle_type.type = cav_msgs::ConnectedVehicleType::CONNECTED;
        obstacle.lanelet_id = lag_veh_start_id;
        obstacle.cross_track = 0.0;
        obstacle.down_track = 0.0;

        std::vector<cav_msgs::RoadwayObstacle> obstacles;
        obstacles.push_back(obstacle);
        cmw->setRoadwayObjects(obstacles);
        cmw->setMap(map);

        EXPECT_TRUE(worker.find_current_gap(obstacle.lanelet_id,obstacle.down_track, req.vehicle_state) > 0.0);
    }

    TEST(CooperativeLaneChangePlugin,TestNoPath_roadwayobject){
        //Tests behavior when there is no path from roadway object to subject vehicle

        // File to process. 
        std::string path = ros::package::getPath("unobstructed_lanechange");
        std::string file = "/resource/map/town01_vector_map_lane_change.osm";
        file = path.append(file);
        lanelet::Id start_id = 107;
        lanelet::Id lane_change_start_id = 106;
        lanelet::Id lag_veh_start_id = 111;     //Lag vehicle at a lanelet from which no path to lane change start lanelet of subject
        lanelet::Id end_id = 111;
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
        lanelet::BasicPoint2d curr_pose = llt.centerline2d().front();

        //Define arguments for function maneuvers_to_points
        cooperative_lanechange::CooperativeLaneChangePlugin worker;
        ros::Time::init(); //Initialize ros::Time

        //get ending downtrack from lanelet id
        double ending_downtrack;
        auto shortest_path = cmw->getRoute()->shortestPath();

        //lanelet::BasicPoint2d veh_pos1 = map.get()->laneletLayer.get(lane_change_start_id).centerline2d()[10];
        lanelet::BasicPoint2d veh_pos= map.get()->laneletLayer.get(lane_change_start_id).centerline2d()[10];
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
        
        cav_srvs::PlanTrajectoryRequest req;
        cav_srvs::PlanTrajectoryResponse resp;
        
        ros::Time::init();
        req.maneuver_plan.planning_start_time = ros::Time::now();
        req.maneuver_plan.planning_completion_time = req.maneuver_plan.planning_start_time + ros::Duration(10.0);
        req.vehicle_state.X_pos_global = veh_pos.x();
        req.vehicle_state.Y_pos_global = veh_pos.y();
        req.vehicle_state.longitudinal_vel = maneuver.lane_change_maneuver.start_speed;

        std::vector<cav_msgs::Maneuver> maneuvers_msg;  
        //Define lane change maneuver

        maneuvers_msg.push_back(maneuver);
        req.maneuver_plan.maneuvers = maneuvers_msg;
        //Add roadway object - lag vehicle
        cav_msgs::ExternalObject object;

        // //Test plan trajectory cb
        //Create a roadway object - Here we define an object from which there is no path to the subject
        object.id =1;
        object.object_type = cav_msgs::ExternalObject::SMALL_VEHICLE;
        object.pose.pose.position.x =0.0;
        object.pose.pose.position.y =0.0;
        object.pose.pose.position.z =0.0;
        object.velocity.twist.linear.x = 5.0;
        
        geometry_msgs::Vector3 size;
        size.x = 4;
        size.y = 2;
        size.z = 1;
        
        object.size = size;

        cav_msgs::PredictedState pred;
        auto pred_pose = object.pose.pose;
        pred_pose.position.y += 1;
        pred.predicted_position = pred_pose;
        pred.predicted_position_confidence = 1.0;
        
        object.predictions.push_back(pred);

        cav_msgs::RoadwayObstacle obstacle;
        obstacle.object = object;
        obstacle.connected_vehicle_type.type = cav_msgs::ConnectedVehicleType::CONNECTED;
        obstacle.lanelet_id = lag_veh_start_id;
        obstacle.cross_track = 0.0;
        obstacle.down_track = 0.0;

        std::vector<cav_msgs::RoadwayObstacle> obstacles;
        obstacles.push_back(obstacle);
        cmw->setRoadwayObjects(obstacles);
        cmw->setMap(map);
   
        try{
            worker.find_current_gap(obstacle.lanelet_id,obstacle.down_track,req.vehicle_state);
        }   
        catch(std::exception &ex){
            EXPECT_EQ(ex.what(), std::string("No path exists from roadway object to subject"));
        }

    }



    int main (int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "test_cooperative_lanechange");
        auto res = RUN_ALL_TESTS();
        return res;
    }