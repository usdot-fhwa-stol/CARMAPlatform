/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "platooning_tactical_plugin/platooning_tactical_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(PlatooningTacticalPluginTest, testGetWaypointsInTimeBoundary1)
{
    // compose a list of waypoints spanning 8 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 8.0;
    wp_3.pose.pose.position.x = 24.0;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 8.0;
    wp_4.pose.pose.position.x = 40.0;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 8.0;
    wp_5.pose.pose.position.x = 48.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);

    platooning_tactical_plugin::PlatooningTacticalPlugin tp;
    tp.start_speed = 2.0;
    tp.end_speed = 8.0;
    std::vector<autoware_msgs::Waypoint> res = tp.get_waypoints_in_time_boundary(waypoints, 6.0);
    EXPECT_EQ(4, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res[2].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(24.0, res[2].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res.back().twist.twist.linear.x, 0.01);
    EXPECT_NEAR(40.0, res.back().pose.pose.position.x, 0.01);
}

TEST(PlatooningTacticalPluginTest, testGetWaypointsInTimeBoundary2)
{
    // compose a list of waypoints spaning less than 6 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    platooning_tactical_plugin::PlatooningTacticalPlugin tp;
    tp.start_speed = 2.0;
    tp.end_speed = 8.0;

    std::vector<autoware_msgs::Waypoint> res = tp.get_waypoints_in_time_boundary(waypoints, 6.0);
    EXPECT_EQ(2, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
}

TEST(PlatooningTacticalPluginTest, testGetWaypointsInTimeBoundary3)
{
    // compose a list of waypoints spanning exactly 5 seconds
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 6.0;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 8.0;
    wp_3.pose.pose.position.x = 24.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    platooning_tactical_plugin::PlatooningTacticalPlugin tp;
    tp.start_speed = 2.0;
    tp.end_speed = 8.0;

    std::vector<autoware_msgs::Waypoint> res = tp.get_waypoints_in_time_boundary(waypoints, 5.0);
    EXPECT_EQ(3, res.size());
    EXPECT_NEAR(2.0, res[0].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(0.0, res[0].pose.pose.position.x, 0.01);
    EXPECT_NEAR(4.0, res[1].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(6.0, res[1].pose.pose.position.x, 0.01);
    EXPECT_NEAR(8.0, res[2].twist.twist.linear.x, 0.01);
    EXPECT_NEAR(24.0, res[2].pose.pose.position.x, 0.01);
}

TEST(PlatooningTacticalPluginTest, testCreateUnevenTrajectory1)
{
    // compose a list of waypoints, uneven spaced
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 0.5;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 2.0;
    wp_3.pose.pose.position.x = 1.3;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 4.0;
    wp_4.pose.pose.position.x = 1.4;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 4.0;
    wp_5.pose.pose.position.x = 2.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);
    platooning_tactical_plugin::PlatooningTacticalPlugin tp;

    tp.start_speed = 2.0;
    tp.end_speed = 8.0;

    // create pose message to indicate that the current location is on top of the starting waypoint
    tp.pose_msg_.reset(new geometry_msgs::PoseStamped());
    std::vector<cav_msgs::TrajectoryPlanPoint> traj = tp.create_uneven_trajectory_from_waypoints(waypoints);
    EXPECT_EQ(5, traj.size());
    EXPECT_NEAR(0.0, traj[0].target_time.toSec(), 0.01);
    EXPECT_NEAR(0.0, traj[0].x, 0.01);
    EXPECT_NEAR(0.25, traj[1].target_time.toSec(), 0.01);
    EXPECT_NEAR(0.5, traj[1].x, 0.01);
    EXPECT_NEAR(0.45, traj[2].target_time.toSec(), 0.01);
    EXPECT_NEAR(1.3, traj[2].x, 0.01);
    EXPECT_NEAR(0.5, traj[3].target_time.toSec(), 0.01);
    EXPECT_NEAR(1.4, traj[3].x, 0.01);
    EXPECT_NEAR(0.65, traj[4].target_time.toSec(), 0.01);
    EXPECT_NEAR(2.0, traj[4].x, 0.01);
}

TEST(PlatooningTacticalPluginTest, testCreateUnevenTrajectory2)
{
    // compose a list of waypoints, uneven spaced
    std::vector<autoware_msgs::Waypoint> waypoints;
    autoware_msgs::Waypoint wp_1;
    wp_1.twist.twist.linear.x = 2.0;
    wp_1.pose.pose.position.x = 0.0;
    autoware_msgs::Waypoint wp_2;
    wp_2.twist.twist.linear.x = 4.0;
    wp_2.pose.pose.position.x = 0.5;
    autoware_msgs::Waypoint wp_3;
    wp_3.twist.twist.linear.x = 2.0;
    wp_3.pose.pose.position.x = 1.3;
    autoware_msgs::Waypoint wp_4;
    wp_4.twist.twist.linear.x = 4.0;
    wp_4.pose.pose.position.x = 1.4;
    autoware_msgs::Waypoint wp_5;
    wp_5.twist.twist.linear.x = 4.0;
    wp_5.pose.pose.position.x = 2.0;
    waypoints.push_back(wp_1);
    waypoints.push_back(wp_2);
    waypoints.push_back(wp_3);
    waypoints.push_back(wp_4);
    waypoints.push_back(wp_5);
    platooning_tactical_plugin::PlatooningTacticalPlugin tp;
    tp.start_speed = 2.0;
    tp.end_speed = 8.0;

    // create pose message to indicate that the current location is not near the starting waypoint
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    tp.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));
    std::vector<cav_msgs::TrajectoryPlanPoint> traj = tp.create_uneven_trajectory_from_waypoints(waypoints);
    EXPECT_EQ(6, traj.size());
    EXPECT_NEAR(0.0, traj[0].target_time.toSec(), 0.01);
    EXPECT_NEAR(-1.0, traj[0].x, 0.01);
    EXPECT_NEAR(0.5, traj[1].target_time.toSec(), 0.01);
    EXPECT_NEAR(0.0, traj[1].x, 0.01);
    EXPECT_NEAR(0.75, traj[2].target_time.toSec(), 0.001);
    EXPECT_NEAR(0.5, traj[2].x, 0.01);
    EXPECT_NEAR(0.95, traj[3].target_time.toSec(), 0.001);
    EXPECT_NEAR(1.3, traj[3].x, 0.01);
    EXPECT_NEAR(1.0, traj[4].target_time.toSec(), 0.001);
    EXPECT_NEAR(1.4, traj[4].x, 0.01);
    EXPECT_NEAR(1.15, traj[5].target_time.toSec(), 0.001);
    EXPECT_NEAR(2.0, traj[5].x, 0.01);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}