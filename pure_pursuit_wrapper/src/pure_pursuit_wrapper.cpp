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

#include "pure_pursuit_wrapper/pure_pursuit_wrapper.hpp"
#include <trajectory_utils/conversions/conversions.h>
#include <carma_wm/Geometry.h>

namespace pure_pursuit_wrapper
{
PurePursuitWrapper::PurePursuitWrapper(WaypointPub waypoint_pub, PluginDiscoveryPub plugin_discovery_pub)
  : waypoint_pub_(waypoint_pub), plugin_discovery_pub_(plugin_discovery_pub)
{
  plugin_discovery_msg_.name = "Pure Pursuit";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
  plugin_discovery_msg_.capability = "control_pure_pursuit_plan/plan_controls";
}

bool PurePursuitWrapper::onSpin()
{
  plugin_discovery_pub_(plugin_discovery_msg_);
  return true;
}

void PurePursuitWrapper::trajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp)
{
  ROS_DEBUG_STREAM("Received TrajectoryPlanCurrentPosecallback message");

  std::vector<double> times;
  std::vector<double> downtracks;
  trajectory_utils::conversions::trajectory_to_downtrack_time(tp->trajectory_points, &downtracks, &times);

  std::vector<double> speeds;
  trajectory_utils::conversions::time_to_speed(downtracks, times, tp->initial_longitudinal_velocity, &speeds);

  if (speeds.size() != tp->trajectory_points.size())
  {
    throw std::invalid_argument("Speeds and trajectory points sizes do not match");
  }

  autoware_msgs::Lane lane;
  lane.header = tp->header;
  std::vector<autoware_msgs::Waypoint> waypoints;
  waypoints.reserve(tp->trajectory_points.size());

  for (int i = 0; i < tp->trajectory_points.size(); i++)
  {
    autoware_msgs::Waypoint wp;

    wp.pose.pose.position.x = tp->trajectory_points[i].x;
    wp.pose.pose.position.y = tp->trajectory_points[i].y;
    wp.twist.twist.linear.x = speeds[i];
    ROS_DEBUG_STREAM("Setting waypoint idx: " << i <<", x: " << tp->trajectory_points[i].x << 
                            ", y: " << tp->trajectory_points[i].y <<
                            ", speed: " << speeds[i]* 2.23694 << "mph");
    waypoints.push_back(wp);
  }

  lane.waypoints = waypoints;
  waypoint_pub_(lane);
};
}  // namespace pure_pursuit_wrapper
