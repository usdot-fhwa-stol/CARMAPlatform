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

#include <gtest/gtest.h>
#include <iostream>
#include <boost/optional/optional.hpp>

TEST(pure_pursuit_wrapper, trajectoryPlanHandler)
{
  cav_msgs::TrajectoryPlan plan;
  plan.initial_longitudinal_velocity = 8.5;

  boost::optional<autoware_msgs::Lane> wp_msg;
  boost::optional<cav_msgs::Plugin> plugin_msg;

  pure_pursuit_wrapper::PurePursuitWrapper ppw([&wp_msg](auto msg) { wp_msg = msg; },
                                               [&plugin_msg](auto msg) { plugin_msg = msg; });

  cav_msgs::TrajectoryPlanPoint tpp, tpp2, tpp3;
  tpp.x = 10;
  tpp.y = 10;
  tpp.target_time = ros::Time(0.1);  // 8.5m/s

  tpp2.x = 12;
  tpp2.y = 12;
  tpp2.target_time = ros::Time(0.2);  // 48.068542495 m/s

  tpp3.x = 14;
  tpp3.y = 14;
  tpp3.target_time = ros::Time(0.3);  // 8.5m/s

  plan.trajectory_points = { tpp, tpp2, tpp3 };

  cav_msgs::TrajectoryPlan::ConstPtr plan_ptr(new cav_msgs::TrajectoryPlan(plan));

  ASSERT_FALSE(!!wp_msg);

  ppw.trajectoryPlanHandler(plan_ptr);

  ASSERT_TRUE(!!wp_msg);

  autoware_msgs::Lane lane = wp_msg.get();

  ASSERT_EQ(3, lane.waypoints.size());
  ASSERT_NEAR(8.5, lane.waypoints[0].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(48.068542495, lane.waypoints[1].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(8.5, lane.waypoints[2].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.y, 0.0000001);
}

TEST(pure_pursuit_wrapper, onSpin)
{
  cav_msgs::TrajectoryPlan plan;
  plan.initial_longitudinal_velocity = 8.5;

  boost::optional<autoware_msgs::Lane> wp_msg;
  boost::optional<cav_msgs::Plugin> plugin_msg;

  pure_pursuit_wrapper::PurePursuitWrapper ppw([&wp_msg](auto msg) { wp_msg = msg; },
                                               [&plugin_msg](auto msg) { plugin_msg = msg; });

  ASSERT_FALSE(!!plugin_msg);

  ASSERT_TRUE(ppw.onSpin());

  ASSERT_TRUE(!!plugin_msg);

  cav_msgs::Plugin msg = plugin_msg.get();
  ASSERT_EQ(0, msg.name.compare("Pure Pursuit"));
  ASSERT_EQ(0, msg.versionId.compare("v1.0"));
  ASSERT_TRUE(msg.available);
  ASSERT_TRUE(msg.activated);
  ASSERT_EQ(cav_msgs::Plugin::CONTROL, msg.type);
  ASSERT_EQ(0, msg.capability.compare("control_pure_pursuit_plan/plan_controls"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
