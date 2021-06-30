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
#include "unobstructed_lanechange.h"
#include <unordered_set>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <carma_utils/containers/containers.h>
#include<limits>

namespace unobstructed_lanechange
{           
    void UnobstructedLaneChangePlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/UnobstructedLaneChangePlugin/plan_trajectory", &UnobstructedLaneChangePlugin::plan_trajectory_cb, this);
        yield_client_ = nh_->serviceClient<cav_srvs::PlanTrajectory>("plugins/YieldPlugin/plan_trajectory");
                
        unobstructed_lanechange_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "UnobstructedLaneChangePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &UnobstructedLaneChangePlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &UnobstructedLaneChangePlugin::twist_cd, this);

        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");
        pnh_->param<double>("minimum_speed", minimum_speed_, 2.2352);
        pnh_->param<double>("max_accel", max_accel_, 1.5);
        pnh_->param<double>("minimum_lookahead_distance", minimum_lookahead_distance_, 5.0);
        pnh_->param<double>("maximum_lookahead_distance", maximum_lookahead_distance_, 25.0);
        pnh_->param<double>("minimum_lookahead_speed", minimum_lookahead_speed_, 2.8);
        pnh_->param<double>("maximum_lookahead_speed", maximum_lookahead_speed_, 13.9);
        pnh_->param<double>("lateral_accel_limit", lateral_accel_limit_, 1.5);
        pnh_->param<int>("speed_moving_average_window_size", speed_moving_average_window_size_, 5);
        pnh_->param<int>("curvature_moving_average_window_size", curvature_moving_average_window_size_, 9);
        pnh_->param<double>("curvature_calc_lookahead_count", curvature_calc_lookahead_count_, 1);
        pnh_->param<int>("downsample_ratio", downsample_ratio_, 8);
        pnh_->param<bool>("enable_object_avoidance_lc", enable_object_avoidance_lc_, false);
        pnh_->param<double>("acceptable_time_difference_", acceptable_time_difference_, 1.0);
        pnh_->param<double>("min_timestep",min_timestep_);
        pnh_->param<int>("turn_downsample_ratio", turn_downsample_ratio_, 0);
        pnh_->param<double>("curve_resample_step_size", curve_resample_step_size_, 0);
        pnh_->param<double>("back_distance", back_distance_, 0.0);
        pnh_->param<double>("buffer_ending_downtrack", buffer_ending_downtrack_, 0);
        // update ros_dur_ value
        time_dur_ = ros::Duration(acceptable_time_difference_);

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { unobstructed_lanechange_plugin_discovery_pub_.publish(plugin_discovery_msg_); });

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();

    }

    void UnobstructedLaneChangePlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    void UnobstructedLaneChangePlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void UnobstructedLaneChangePlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::spin();

    }

    bool UnobstructedLaneChangePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){

        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        //convert maneuver info to route points and speeds
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        if(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan].type != cav_msgs::Maneuver::LANE_CHANGE)
        {
            throw std::invalid_argument ("Unobstructed Lane Change Plugin doesnt support this maneuver type");
        }
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan]);
        resp.related_maneuvers.push_back(req.maneuver_index_to_plan);
        
        basic_autonomy::waypoint_generation::DetailedTrajConfig wpg_detail_config;
        basic_autonomy::waypoint_generation::GeneralTrajConfig wpg_general_config;

        wpg_general_config = basic_autonomy::waypoint_generation::compose_general_trajectory_config("unobstructed_lanechange", downsample_ratio_, turn_downsample_ratio_);

        wpg_detail_config = basic_autonomy::waypoint_generation::compose_detailed_trajectory_config(trajectory_time_length_, 
                                                                            curve_resample_step_size_, minimum_speed_, 
                                                                            max_accel_, lateral_accel_limit_, 
                                                                            speed_moving_average_window_size_, 
                                                                            curvature_moving_average_window_size_, back_distance_,
                                                                            buffer_ending_downtrack_);

        ROS_DEBUG_STREAM("Current downtrack:"<<current_downtrack);
        auto points_and_target_speeds = basic_autonomy::waypoint_generation::maneuvers_to_points_lanechange(maneuver_plan, current_downtrack, wm_, 
                                                                                                    req.vehicle_state, maneuver_fraction_completed_, 
                                                                                                    ending_state_before_buffer_, wpg_detail_config);                                                                                                    
        ROS_DEBUG_STREAM("Maneuvers to points size:"<<points_and_target_speeds.size());
        auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, downsample_ratio_);

        int starting_lanelet_id = stoi(maneuver_plan.front().lane_change_maneuver.starting_lane_id);

        double final_max_speed = maneuver_plan.front().lane_change_maneuver.end_speed;

        cav_msgs::TrajectoryPlan original_trajectory;
        original_trajectory.header.frame_id = "map";
        original_trajectory.header.stamp = ros::Time::now();
        original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        original_trajectory.trajectory_points = basic_autonomy::waypoint_generation::compose_lanechange_trajectory_from_centerline(downsampled_points, req.vehicle_state, req.header.stamp,
                                                                                     wm_, ending_state_before_buffer_, wpg_detail_config);
        ROS_DEBUG_STREAM("Compose Trajectory size:"<<original_trajectory.trajectory_points.size());


        original_trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, minimum_speed_);

        if (enable_object_avoidance_lc_)
        {
            if (yield_client_ && yield_client_.exists() && yield_client_.isValid())
            {
                ROS_DEBUG_STREAM("Yield Client is valid");
                cav_srvs::PlanTrajectory yield_srv;
                yield_srv.request.initial_trajectory_plan = original_trajectory;
                yield_srv.request.vehicle_state = req.vehicle_state;
                if (yield_client_.call(yield_srv))
                {
                    cav_msgs::TrajectoryPlan yield_plan = yield_srv.response.trajectory_plan;
                    if (validate_yield_plan(yield_plan, original_trajectory.trajectory_id))
                    {
                    resp.trajectory_plan = yield_plan;
                    }
                    else
                    {
                    throw std::invalid_argument("Invalid Yield Trajectory");
                    }
                }
                else
                {
                    throw std::invalid_argument("Unable to Call Yield Plugin");
                }
            }
            else
            {
                throw std::invalid_argument("Yield Client is unavailable");
            }
        }
        else
        {
            resp.trajectory_plan = original_trajectory;
        }

        

        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;

    }

   

    bool UnobstructedLaneChangePlugin::validate_yield_plan(const cav_msgs::TrajectoryPlan& yield_plan, const std::string& original_plan_id)
    {
        if (yield_plan.trajectory_points.size()>= 2 && yield_plan.trajectory_id == original_plan_id)
        {
            ros::Duration time_difference = yield_plan.trajectory_points[0].target_time - ros::Time::now();
            
            if (time_difference <= time_dur_)
            {
                return true;
            }
            else
            {
                ROS_DEBUG_STREAM("Old Yield Trajectory");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Invalid Yield Trajectory"); 
        }
        return false;
    }

}