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
#include <ros/ros.h>
#include <string>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "cooperative_lanechange.h"
#include <unordered_set>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <carma_utils/containers/containers.h>
#include <cav_msgs/MobilityPath.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/LaneletMap.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/ConnectedVehicleType.h>

namespace cooperative_lanechange
{
    void CooperativeLaneChangePlugin::initialize()
    {
        //@SONAR_STOP@
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/CooperativeLaneChangePlugin/plan_trajectory", &CooperativeLaneChangePlugin::plan_trajectory_cb, this);
                
        cooperative_lanechange_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "CooperativeLaneChangePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &CooperativeLaneChangePlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &CooperativeLaneChangePlugin::twist_cd, this);
        incoming_mobility_response_ = nh_->subscribe("incoming_mobility_response", 1 , &CooperativeLaneChangePlugin::mobilityresponse_cb, this);
        
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &CooperativeLaneChangePlugin::bsm_cb, this);
        outgoing_mobility_request_ = nh_->advertise<cav_msgs::MobilityRequest>("outgoing_mobility_request", 5); // rate from yield plugin
        lanechange_status_pub_ = nh_->advertise<cav_msgs::LaneChangeStatus>("cooperative_lane_change_status",10);
        //Vehicle params
        pnh_->getParam("vehicle_id",sender_id_);

        //Plugin params
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");
        pnh_->param<double>("minimum_speed", minimum_speed_, 2.2352);
        pnh_->param<double>("max_accel", max_accel_, 1.5);
        pnh_->param<double>("minimum_lookahead_distance", minimum_lookahead_distance_, 5.0);
        pnh_->param<double>("maximum_lookahead_distance", maximum_lookahead_distance_, 25.0);
        pnh_->param<double>("minimum_lookahead_speed", minimum_lookahead_speed_, 2.8);
        pnh_->param<double>("maximum_lookahead_speed", maximum_lookahead_speed_, 13.9);
        pnh_->param<double>("lateral_accel_limit", lateral_accel_limit_, 1.5);
        pnh_->param<double>("speed_moving_average_window_size", speed_moving_average_window_size_, 5);
        pnh_->param<double>("curvature_moving_average_window_size", curvature_moving_average_window_size_, 5);
        pnh_->param<double>("curvature_calc_lookahead_count", curvature_calc_lookahead_count_, 1);
        pnh_->param<int>("downsample_ratio", downsample_ratio_, 8);
        pnh_->param<double>("destination_range",destination_range_, 5);
        pnh_->param<double>("lanechange_time_out",lanechange_time_out_, 6.0);
        pnh_->param<double>("min_timestep",min_timestep_);
        pnh_->param<double>("starting_downtrack_range", starting_downtrack_range_, 5.0);
        pnh_->param<double>("starting_fraction", starting_fraction_, 0.2);
        pnh_->param<double>("mid_fraction",mid_fraction_, 0.5);
        pnh_->param<double>("min_desired_gap",min_desired_gap_, 5.0);
        pnh_->param<int>("turn_downsample_ratio", turn_downsample_ratio_, 0);
        pnh_->param<double>("curve_resample_step_size", curve_resample_step_size_, 0);
        pnh_->param<double>("back_distance", back_distance_, 0.0);
        pnh_->param<double>("buffer_ending_downtrack", buffer_ending_downtrack_, 0);

        //tf listener for looking up earth to map transform 
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { cooperative_lanechange_plugin_discovery_pub_.publish(plugin_discovery_msg_); });

        //@SONAR_START@
    }

    void CooperativeLaneChangePlugin::mobilityresponse_cb(const cav_msgs::MobilityResponse &msg){
        //@SONAR_STOP@
        cav_msgs::LaneChangeStatus lc_status_msg;
        if(msg.is_accepted){
            is_lanechange_accepted_ = true;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::ACCEPTANCE_RECEIVED;
            lc_status_msg.description = "Received lane merge acceptance";
            
        }
        else{
            is_lanechange_accepted_ = false;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::REJECTION_RECEIVED;
            lc_status_msg.description = "Received lane merge rejection";
        }
        lanechange_status_pub_.publish(lc_status_msg);
        //@SONAR_START@
    }


    double CooperativeLaneChangePlugin::find_current_gap(long veh2_lanelet_id, double veh2_downtrack) const {
                
        //find downtrack distance between ego and lag vehicle
        ROS_DEBUG_STREAM("entered find_current_gap");
        double current_gap = 0.0;
        lanelet::BasicPoint2d ego_pos(pose_msg_.pose.position.x,pose_msg_.pose.position.y);
        //double ego_current_downtrack = wm_->routeTrackPos(ego_pos).downtrack;
        
        lanelet::LaneletMapConstPtr const_map(wm_->getMap());
        lanelet::ConstLanelet veh2_lanelet = const_map->laneletLayer.get(veh2_lanelet_id);
        ROS_DEBUG_STREAM("veh2_lanelet id " << veh2_lanelet.id());

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(const_map->laneletLayer, current_loc, 10);       
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
        ROS_DEBUG_STREAM("current llt id " << current_lanelet.id());
        
        //Create temporary route between the two vehicles
        lanelet::ConstLanelet start_lanelet = veh2_lanelet;
        lanelet::ConstLanelet end_lanelet = current_lanelet;
        lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany,lanelet::Participants::VehicleCar);
        ROS_DEBUG_STREAM("traffic rules created]");
        
        lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*(wm_->getMap()), *traffic_rules);
        ROS_DEBUG_STREAM("Graph created");

        auto temp_route = map_graph->getRoute(start_lanelet, end_lanelet);
        ROS_DEBUG_STREAM("Route created");
        
        auto shortest_path2 = temp_route.get().shortestPath();
        ROS_DEBUG_STREAM("Shorted path created size: " << shortest_path2.size());
        for (auto llt : shortest_path2)
        {
            ROS_DEBUG_STREAM("llt id  route: " << llt.id());
        }  


        //To find downtrack- creating temporary route from veh2 to veh1(ego vehicle)
        
        double veh1_current_downtrack = wm_->routeTrackPos(ego_pos).downtrack;      
        ROS_DEBUG_STREAM("ego_current_downtrack:" << veh1_current_downtrack);
        
        
        current_gap = veh1_current_downtrack - veh2_downtrack;
        ROS_DEBUG_STREAM("Finding current gap");
        ROS_DEBUG_STREAM("Veh1 current downtrack:"<<veh1_current_downtrack<<" veh2 downtrack:"<<veh2_downtrack);
        

        return current_gap;

    }

    void CooperativeLaneChangePlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void CooperativeLaneChangePlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }
    
    void CooperativeLaneChangePlugin::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        bsm_core_ = msg->core_data;
        
    }

    void CooperativeLaneChangePlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::spin();

    }

    

    bool CooperativeLaneChangePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        //@SONAR_STOP@
        //  Only plan the trajectory for the requested LANE_CHANGE maneuver
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        if(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan].type != cav_msgs::Maneuver::LANE_CHANGE)
        {
            throw std::invalid_argument ("Cooperative Lane Change Plugin doesn't support this maneuver type");
        }
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan]);

        //Current only checking for first lane change maneuver message
        long target_lanelet_id = stol(maneuver_plan[0].lane_change_maneuver.ending_lane_id);
        double target_downtrack = maneuver_plan[0].lane_change_maneuver.end_dist;
        //get subject vehicle info
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
        if(current_downtrack < maneuver_plan[0].lane_change_maneuver.start_dist - starting_downtrack_range_){
            return true;
        }
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, veh_pos, 10);       
        long current_lanelet_id = current_lanelets[0].second.id();
        if(current_lanelet_id == target_lanelet_id && current_downtrack >= target_downtrack - destination_range_){
            cav_msgs::LaneChangeStatus lc_status_msg;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::PLANNING_SUCCESS;
            //No description as per UI documentation
            lanechange_status_pub_.publish(lc_status_msg);
        }

        long veh2_lanelet_id = 0;
        double veh2_downtrack = 0.0, veh2_speed = 0.0;
        bool foundRoadwayObject = false;
        bool negotiate = true;
        std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
        //Assuming only one connected vehicle in list 
        for(int i=0;i<rwol.size();i++){
            if(rwol[i].connected_vehicle_type.type == cav_msgs::ConnectedVehicleType::NOT_CONNECTED){
                veh2_lanelet_id = rwol[0].lanelet_id;
                veh2_downtrack = rwol[0].down_track; //Returns downtrack
                veh2_speed = rwol[0].object.velocity.twist.linear.x;
                foundRoadwayObject = true;
                break;
            }
        }
        if(foundRoadwayObject){
            ROS_DEBUG_STREAM("Found Roadway object");
            //get current_gap
            ROS_DEBUG_STREAM("veh2_lanelet_id:" << veh2_lanelet_id << "veh2_downtrack" << veh2_downtrack);
            
            double current_gap = find_current_gap(veh2_lanelet_id,veh2_downtrack);
            ROS_DEBUG_STREAM("Current gap:"<<current_gap);
            //get desired gap - desired time gap (default 3s)* relative velocity
            double relative_velocity = current_speed_ - veh2_speed;
            ROS_DEBUG_STREAM("Relative velocity:"<<relative_velocity);
            double desired_gap = desired_time_gap_ * relative_velocity;      
            ROS_DEBUG_STREAM("Desired gap:"<<desired_gap);
            if(desired_gap < 5.0){
                desired_gap = 5.0;
            }
            // if(current_gap > desired_gap){
            //     negotiate = false;  //No need for negotiation
            // }
            
        }
        else{
            ROS_DEBUG_STREAM("No roadway object");
            //ROS_WARN_STREAM("Did not find a connected and automated vehicle roadway object");
            negotiate = false;
        }

        //plan lanechange without filling in response
        ROS_DEBUG_STREAM("Planning lane change trajectory");
        std::vector<cav_msgs::TrajectoryPlanPoint> planned_trajectory_points = plan_lanechange(req);
        
        if(negotiate){
            ROS_DEBUG_STREAM("Negotiating");
            //send mobility request
            //Planning for first lane change maneuver
            cav_msgs::MobilityRequest request = create_mobility_request(planned_trajectory_points, maneuver_plan[0]);
            outgoing_mobility_request_.publish(request);
            if(!request_sent){
                request_sent_time = ros::Time::now();
                request_sent = true;
            }
            cav_msgs::LaneChangeStatus lc_status_msg;
            lc_status_msg.status = cav_msgs::LaneChangeStatus::PLAN_SENT;
            lc_status_msg.description = "Requested lane merge";
            lanechange_status_pub_.publish(lc_status_msg);
        }

        //if ack mobility response, send lanechange response
        if(!negotiate || is_lanechange_accepted_){
            ROS_DEBUG_STREAM("negotiate:" << negotiate);
            ROS_DEBUG_STREAM("is_lanechange_accepted:" << is_lanechange_accepted_);

            ROS_DEBUG_STREAM("Adding to response");
            add_maneuver_to_response(req,resp,planned_trajectory_points);
            
        }
        else{
            if(!negotiate && !request_sent){
                request_sent_time = ros::Time::now();
                request_sent = true;
            }
            ros::Time planning_end_time = ros::Time::now();
            ros::Duration passed_time = planning_end_time - request_sent_time;
            if(passed_time.toSec() >= lanechange_time_out_){
                cav_msgs::LaneChangeStatus lc_status_msg;
                lc_status_msg.status = cav_msgs::LaneChangeStatus::TIMED_OUT;
                lc_status_msg.description = "Request timed out for lane merge";
                lanechange_status_pub_.publish(lc_status_msg);
                request_sent = false;  //Reset variable
            }
 
        }
        //@SONAR_START@
        return true;
    }

    void CooperativeLaneChangePlugin::add_maneuver_to_response(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp, std::vector<cav_msgs::TrajectoryPlanPoint> &planned_trajectory_points){
        cav_msgs::TrajectoryPlan trajectory_plan;
        trajectory_plan.header.frame_id = "map";
        trajectory_plan.header.stamp = ros::Time::now();
        trajectory_plan.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        trajectory_plan.trajectory_points = planned_trajectory_points;
        trajectory_plan.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, minimum_speed_);
        resp.trajectory_plan = trajectory_plan;

        resp.related_maneuvers.push_back(req.maneuver_index_to_plan);

        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
    }
    
    cav_msgs::MobilityRequest CooperativeLaneChangePlugin::create_mobility_request(std::vector<cav_msgs::TrajectoryPlanPoint>& trajectory_plan, cav_msgs::Maneuver& maneuver){

        cav_msgs::MobilityRequest request_msg;
        cav_msgs::MobilityHeader header;
        header.sender_id = sender_id_;
        header.recipient_id = DEFAULT_STRING_;  
        header.sender_bsm_id = bsmIDtoString(bsm_core_);
        header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
        header.timestamp = trajectory_plan.front().target_time.toNSec() *1000000;
        request_msg.header = header;

        request_msg.strategy = "carma/cooperative-lane-change";
        request_msg.plan_type.type = cav_msgs::PlanType::CHANGE_LANE_LEFT;
        //Urgency- Currently unassigned
        int urgency;
        if(maneuver_fraction_completed_ <= starting_fraction_){
            urgency = 10;
        }
        else if(maneuver_fraction_completed_ <= mid_fraction_ ){
            urgency = 5;
        }
        else{
            urgency=1;
        }
        ROS_DEBUG_STREAM("Maneuver fraction completed:"<<maneuver_fraction_completed_);
        request_msg.urgency = urgency;

        //Strategy params
        //Encode JSON with Boost Property Tree
        using boost::property_tree::ptree;
        ptree pt;
        double end_speed_floor = std::floor(maneuver.lane_change_maneuver.end_speed);
        int end_speed_fractional = (maneuver.lane_change_maneuver.end_speed - end_speed_floor) * 10;

        ROS_DEBUG_STREAM("end_speed_floor" << end_speed_floor);
        ROS_DEBUG_STREAM("end_speed_fractional" << end_speed_fractional);
        ROS_DEBUG_STREAM("start_lanelet_id" << maneuver.lane_change_maneuver.starting_lane_id);
        ROS_DEBUG_STREAM("end_lanelet_id" << maneuver.lane_change_maneuver.ending_lane_id);

        pt.put("s",(int)end_speed_floor); 
        pt.put("f",end_speed_fractional); 
        pt.put("sl",maneuver.lane_change_maneuver.starting_lane_id);
        pt.put("el", maneuver.lane_change_maneuver.ending_lane_id);

        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream,pt);
        request_msg.strategy_params = body_stream.str(); 
        ROS_DEBUG_STREAM("request_msg.strategy_params" << request_msg.strategy_params);

        //Trajectory
        cav_msgs::Trajectory trajectory;
        //get earth to map tf
        try
        {
            geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform("earth", "map", ros::Time(0));
            trajectory = trajectory_plan_to_trajectory(trajectory_plan, tf);
            //Location
            cav_msgs::TrajectoryPlanPoint temp_loc_to_convert;
            temp_loc_to_convert.x = pose_msg_.pose.position.x;
            temp_loc_to_convert.y = pose_msg_.pose.position.y;
            tf2::Stamped<tf2::Transform> transform;
            tf2::fromMsg(tf, transform);
            cav_msgs::LocationECEF location = trajectory_point_to_ecef(temp_loc_to_convert, transform);

            //Using trajectory first point time as location timestamp
            location.timestamp = trajectory_plan.front().target_time.toNSec() * 1000000;

            request_msg.location = location;

        }
        catch (tf2::TransformException &ex)
        {
            //Throw exception
            ROS_WARN("%s", ex.what());
        }

        request_msg.trajectory = trajectory;
        request_msg.expiration = trajectory_plan.back().target_time.toSec();
        ROS_DEBUG_STREAM("request_msg.expiration: " << request_msg.expiration << "of which string size: " << std::to_string(request_msg.expiration).size());
        
        return request_msg;
    }

    cav_msgs::Trajectory CooperativeLaneChangePlugin::trajectory_plan_to_trajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, const geometry_msgs::TransformStamped& tf) const{
        cav_msgs::Trajectory traj;

        tf2::Stamped<tf2::Transform> transform;
        tf2::fromMsg(tf, transform);

        cav_msgs::LocationECEF ecef_location = trajectory_point_to_ecef(traj_points[0], transform);

        if (traj_points.size()<2){
            ROS_WARN("Received Trajectory Plan is too small");
            traj.offsets = {};
        }
         else{
            cav_msgs::LocationECEF prev_point = ecef_location;
            for (size_t i=1; i<traj_points.size(); i++){
                
                cav_msgs::LocationOffsetECEF offset;
                cav_msgs::LocationECEF new_point = trajectory_point_to_ecef(traj_points[i], transform); //m to cm to fit the msg standard
                offset.offset_x = (int16_t)(new_point.ecef_x - prev_point.ecef_x);  
                offset.offset_y = (int16_t)(new_point.ecef_y - prev_point.ecef_y);
                offset.offset_z = (int16_t)(new_point.ecef_z - prev_point.ecef_z);
                prev_point = new_point;
                traj.offsets.push_back(offset);
            }
        }

        traj.location = ecef_location; 

        return traj;
    }

    cav_msgs::LocationECEF CooperativeLaneChangePlugin::trajectory_point_to_ecef(const cav_msgs::TrajectoryPlanPoint& traj_point, const tf2::Transform& transform) const{
        cav_msgs::LocationECEF ecef_point;    
        
        auto traj_point_vec = tf2::Vector3(traj_point.x, traj_point.y, 0.0);
        tf2::Vector3 ecef_point_vec = transform * traj_point_vec;
        ecef_point.ecef_x = (int32_t)(ecef_point_vec.x() * 100.0); // m to cm
        ecef_point.ecef_y = (int32_t)(ecef_point_vec.y() * 100.0);
        ecef_point.ecef_z = (int32_t)(ecef_point_vec.z() * 100.0); 

        return ecef_point;
    } 

    std::vector<cav_msgs::TrajectoryPlanPoint> CooperativeLaneChangePlugin::plan_lanechange(cav_srvs::PlanTrajectoryRequest &req){
        
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        // Only plan the trajectory for the requested LANE_CHANGE maneuver
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        if(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan].type != cav_msgs::Maneuver::LANE_CHANGE)
        {
            throw std::invalid_argument ("Cooperative Lane Change Plugin doesn't support this maneuver type");
        }
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan]);

        if(current_downtrack >= maneuver_plan.front().lane_change_maneuver.end_dist){
            request_sent = false;
        }
        basic_autonomy::waypoint_generation::DetailedTrajConfig wpg_detail_config;
        basic_autonomy::waypoint_generation::GeneralTrajConfig wpg_general_config;

        wpg_general_config = basic_autonomy::waypoint_generation::compose_general_trajectory_config("cooperative_lanechange", downsample_ratio_, turn_downsample_ratio_);
        
        wpg_detail_config = basic_autonomy::waypoint_generation::compose_detailed_trajectory_config(trajectory_time_length_, 
                                                                            curve_resample_step_size_, minimum_speed_, 
                                                                            max_accel_, lateral_accel_limit_, 
                                                                            speed_moving_average_window_size_, 
                                                                            curvature_moving_average_window_size_, back_distance_,
                                                                            buffer_ending_downtrack_);

        ROS_DEBUG_STREAM("Current downtrack:"<<current_downtrack);
        //auto points_and_target_speeds = maneuvers_to_points(maneuver_plan, current_downtrack, wm_,req.vehicle_state);
        auto points_and_target_speeds = basic_autonomy::waypoint_generation::maneuvers_to_points_lanechange(maneuver_plan, current_downtrack, wm_, 
                                                                                                    req.vehicle_state, maneuver_fraction_completed_, 
                                                                                                    ending_state_before_buffer_, wpg_detail_config);
        ROS_DEBUG_STREAM("Maneuvers to points size:"<<points_and_target_speeds.size());
        auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, downsample_ratio_);

        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points = basic_autonomy::waypoint_generation::compose_lanechange_trajectory_from_centerline(downsampled_points, req.vehicle_state, req.header.stamp,
                                                                                         wm_, ending_state_before_buffer_, wpg_detail_config);
        ROS_DEBUG_STREAM("Compose Trajectory size:"<<trajectory_points.size());
        return trajectory_points;

    }
   
}

