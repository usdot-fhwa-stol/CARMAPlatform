/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include "mpc_follower_wrapper/mpc_follower_wrapper.hpp"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

namespace mpc_follower_wrapper {

MPCFollowerWrapper::MPCFollowerWrapper(ros::CARMANodeHandle &nodeHandle): nh_(nodeHandle)
{

  Initialize();

  ROS_INFO("Successfully launched node.");
}

void MPCFollowerWrapper::Initialize() {


  // Trajectory Plan Subscriber
  
  trajectory_plan_sub = nh_.subscribe<cav_msgs::TrajectoryPlan>("mpc_follower/trajectory", 1, &MPCFollowerWrapper::TrajectoryPlanPoseHandler, this);

  // WayPoints Publisher
  way_points_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);

  mpc_plugin_discovery_pub_ = nh_.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
  plugin_discovery_msg_.name = "MPC";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
  plugin_discovery_msg_.capability = "control_mpc_plan/plan_controls";

  ros::CARMANodeHandle::setSpinCallback([this]() {
    mpc_plugin_discovery_pub_.publish(plugin_discovery_msg_);
    return true;
  });
}

void MPCFollowerWrapper::CurrentPoseHandler(const geometry_msgs::PoseStamped::ConstPtr& pose) {
  current_pose_ = *pose;
}


void MPCFollowerWrapper::TrajectoryPlanPoseHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");
    try {
      autoware_msgs::Lane lane;
      lane.header = tp->header;
      std::vector <autoware_msgs::Waypoint> waypoints;

      for(int i = 0; i < tp->trajectory_points.size() - 1; i++ ) {

        cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[i];
        cav_msgs::TrajectoryPlanPoint t2 = tp->trajectory_points[i + 1];
        autoware_msgs::Waypoint waypoint = mpcww.TrajectoryPlanPointToWaypointConverter(t1, t2);

        waypoints.push_back(waypoint);
      }

      lane.waypoints = waypoints;
      PublisherForWayPoints(lane);
    }
    catch(const std::exception& e) {
      ros::CARMANodeHandle::handleException(e);
    }

};


void MPCFollowerWrapper::PublisherForWayPoints(const autoware_msgs::Lane& msg){
  way_points_pub_.publish(msg);
};

}  // namespace mpc_follower_wrapper
