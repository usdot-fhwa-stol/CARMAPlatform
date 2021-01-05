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

#include <ros/ros.h>
#include <carma_utils/CARMANodeHandle.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit_wrapper_node");
  ros::CARMANodeHandle nh;

  ros::Publisher waypoints_pub = nh.advertise<autoware_msgs::Lane>("final_waypoints", 10, true);

  ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);

  pure_pursuit_wrapper::PurePursuitWrapper purePursuitWrapper(
      [&waypoints_pub](auto msg) { waypoints_pub.publish(msg); },
      [&discovery_pub](auto msg) { discovery_pub.publish(msg); });

  // Trajectory Plan Subscriber
  ros::Subscriber trajectory_plan_sub = nh.subscribe(
      "pure_pursuit/plan_trajectory", 1, &pure_pursuit_wrapper::PurePursuitWrapper::trajectoryPlanHandler, &purePursuitWrapper);

  ros::CARMANodeHandle::setSpinRate(10);

  ros::CARMANodeHandle::setSpinCallback(std::bind(&pure_pursuit_wrapper::PurePursuitWrapper::onSpin, &purePursuitWrapper));

  ROS_INFO("Successfully launched node.");
  ros::CARMANodeHandle::spin();

  return 0;
}