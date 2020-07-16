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
#include "route_following_plugin.h"


namespace route_following_plugin
{
    RouteFollowingPlugin::RouteFollowingPlugin() : mvr_duration_(16.0), current_speed_(0.0) { }

    void RouteFollowingPlugin::initialize()
    {
        
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        plan_maneuver_srv_ = nh_->advertiseService("strategic_plan/plan_maneuvers", &RouteFollowingPlugin::plan_maneuver_cb, this);
                
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "RouteFollowingPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        pose_sub_ = nh_->subscribe("current_pose", 1, &RouteFollowingPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &RouteFollowingPlugin::twist_cd, this);
        
        pnh_->param<double>("minimal_maneuver_duration", mvr_duration_, 16.0);

        wml_.reset(new carma_wm::WMListener());
        // set world model point form wm listener
        wm_ = wml_->getWorldModel();

        ros::CARMANodeHandle::setSpinCallback([this]()
        {
            plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }

    void RouteFollowingPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    bool RouteFollowingPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {        

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 1);
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        auto current_lanelet = current_lanelets[0];
        auto shortest_path = wm_->getRoute()->shortestPath();
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        double total_maneuver_length = current_progress + mvr_duration_ * RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS;
        int last_lanelet_index = findLaneletIndexFromPath(current_lanelet.second.id(), shortest_path);
        if(last_lanelet_index == -1)
        {
            ROS_WARN_STREAM("Cannot find current lanelet in shortest path!");
            return true;
        }
        while(current_progress < total_maneuver_length && last_lanelet_index < shortest_path.size())
        {
            double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
            double dist_diff = end_dist - current_progress;

            resp.new_plan.maneuvers.push_back(
                composeManeuverMessage(current_progress, end_dist, 
                                       speed_progress, RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS, 
                                       shortest_path[last_lanelet_index].id(), ros::Time::now()));

            current_progress += dist_diff;
            speed_progress = RouteFollowingPlugin::TWENTY_FIVE_MPH_IN_MS;
            if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
            {
                break;
            }

            auto following_lanelets = wm_->getRoute()->followingRelations(shortest_path[last_lanelet_index]);
            if(following_lanelets.size() == 0)
            {
                ROS_WARN_STREAM("Cannot find the following lanelet.");
                return true;
            }
            if(identifyLaneChange(following_lanelets, shortest_path[last_lanelet_index + 1].id()))
            {
                ++last_lanelet_index;
            }
            else
            {
                ROS_WARN_STREAM("Cannot find the next lanelet in the current lanelet's successor list!");
                return true;
            }
        }
        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }
        return true;
    }

    void RouteFollowingPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }

    void RouteFollowingPlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    int RouteFollowingPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    cav_msgs::Maneuver RouteFollowingPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InlaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * (current_speed + target_speed)));
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        return maneuver_msg;
    }

    bool RouteFollowingPlugin::identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id)
    {
        for(auto& relation : relations)
        {
            if(relation.lanelet.id() == target_id && relation.relationType == lanelet::routing::RelationType::Successor)
            {
                return true;
            }
        }
        return false;
    }

}