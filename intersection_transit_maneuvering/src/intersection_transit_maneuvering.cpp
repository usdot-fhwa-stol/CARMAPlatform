#pragma once

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
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <intersection_transit_maneuvering.h>

using oss = std::ostringstream;

namespace intersection_transit_maneuvering
{
    IntersectionTransitManeuvering::IntersectionTransitManeuvering();


    void IntersectionTransitManeuvering::initialize()
    {
         nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh2_.reset(new ros::CARMANodeHandle("/"));

        trajectory_srv_ = nh_->advertiseService("plan_trajectory",&IntersectionTransitManeuvering::plan_trajectory_cb, this);
        
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery",1);
        jerk_pub_ = nh_->advertise<std_msgs::Float64>("jerk",1);
        plugin_discovery_msg_.name = "IntersectionTransitManeuvering";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose",1, &IntersectionTransitManeuvering::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &IntersectionTransitManeuvering::twist_cb, this);

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();
        
        pnh_->param<double>("minimal_trajectory_duration", minimal_trajectory_duration_);
        pnh_->param<double>("max_jerk_limit", max_jerk_limit_);
        pnh_->param<double>("min_timestep",min_timestep_);
        pnh_->param<double>("min_jerk", min_jerk_limit_);

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { 
                plugin_discovery_pub_.publish(plugin_discovery_msg_);
                std_msgs::Float64 jerk_msg;
                jerk_msg.data = jerk_;
                jerk_pub_.publish(jerk_msg);
             });
    }



    bool IntersectionTransitManeuvering::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        
        ros::WallTime start_time = ros::WallTime::now();  // Start timeing the execution time for planning so it can be logged

        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
        {
            if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT || 
            req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT )
            {
                maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
                resp.related_maneuvers.push_back(i);
            }
            else
                {
                    break;
                }
        }
        cav_srvs::PlanTrajectoryRequest& req2;
        auto new_plan = convert_maneuver_plan(maneuver_plan);

        for(const auto& man : new_plan)
            {
                req2.maneuver_plan.push_back(man);
            }

        /*Once the maneuvers have been successfully converted, call the inlanecruising plugin to calculate the trajectory*/
        inlanecruising_plugin::InLaneCruisingPlugin::plan_trajectory_cb(req2, resp);
        
        return true;
    }
    

    void IntersectionTransitManeuvering::run()
    {
        /**/
        initialize();
        ros::CARMANodeHandle::spin();
    
    }

   
    std::vector<cav_msgs::Maneuver> IntersectionTransitManeuvering::convert_maneuver_plan(const std::vector<cav_msgs::Maneuver>& maneuvers)
    {
        if (maneuvers.size().isEmpty())
        {
            throw std::invalid_argument("No maneuvers to convert");
        }

        std::vector<cav_msgs::LaneFollowingManeuver> new_maneuver_plan;
        cav_msgs:LaneFollowingManeuver new_maneuver;
        for(const auto& maneuver : maneuvers)
        {
            /*Throw exception if the manuever type does not match INTERSECTION_TRANSIT*/
            if ( maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ||
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ||
                maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ||)
                {
                    throw std::invalid_argument("Intersection transit maneuvering does not support this maneuver type");
                }
            
            /*Convert IT Straight*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT)
            {
                new_maneuver.parameters = maneuver.intersection_transit_straight_maneuver.parameters;

                new_maneuver.start_dist = maneuver.intersection_transit_straight_maneuver.start_dist;
                new_maneuver.start_speed = maneuver.intersection_transit_straight_maneuver.start_speed;
                new_maneuver.start_time = maneuver.intersection_transit_straight_maneuver.start_time;

                new_maneuver.end_dist = maneuver.intersection_transit_straight_maneuver.end_dist;
                new_maneuver.end_speed = maneuver.intersection_transit_straight_maneuver.end_speed;
                new_maneuver.end_time = maneuver.intersection_transit_straight_maneuver.end_time;

                new_maneuver.lane_id = maneuver.intersection_transit_straight_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT LEFT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                new_maneuver.parameters = maneuver.intersection_transit_left_turn_maneuver.parameters;

                new_maneuver.start_dist = maneuver.intersection_transit_left_turn_maneuver.start_dist;
                new_maneuver.start_speed = maneuver.intersection_transit_left_turn_maneuver.start_speed;
                new_maneuver.start_time = maneuver.intersection_transit_left_turn_maneuver.start_time;

                new_maneuver.end_dist = maneuver.intersection_transit_left_turn_maneuver.end_dist;
                new_maneuver.end_speed = maneuver.intersection_transit_left_turn_maneuver.end_speed;
                new_maneuver.end_time = maneuver.intersection_transit_left_turn_maneuver.end_time;

                new_maneuver.lane_id = maneuver.intersection_transit_left_turn_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }

             /*Convert IT RIGHT TURN*/
            if (maneuver.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN)
            {
                new_maneuver.parameters = maneuver.intersection_transit_right_turn_maneuver.parameters;

                new_maneuver.start_dist = maneuver.intersection_transit_right_turn_maneuver.start_dist;
                new_maneuver.start_speed = maneuver.intersection_transit_right_turn_maneuver.start_speed;
                new_maneuver.start_time = maneuver.intersection_transit_right_turn_maneuver.start_time;

                new_maneuver.end_dist = maneuver.intersection_transit_right_turn_maneuver.end_dist;
                new_maneuver.end_speed = maneuver.intersection_transit_right_turn_maneuver.end_speed;
                new_maneuver.end_time = maneuver.intersection_transit_right_turn_maneuver.end_time;

                new_maneuver.lane_id = maneuver.intersection_transit_right_turn_maneuver.lane_id;

                new_maneuver_plan.push_back(new_maneuver);
            }
  
        }//end for-loop

        return new_maneuver_plan;

    }






}