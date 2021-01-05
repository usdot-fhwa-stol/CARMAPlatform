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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/CARMAWorldModel.h>
#include <math.h>

using namespace inlanecruising_plugin;
// Test to ensure Eigen::Isometry2d behaves like tf2::Transform
TEST(InLaneCruisingPluginTest, validate_eigen)
{
  Eigen::Rotation2Dd frame_rot(M_PI_2);
  lanelet::BasicPoint2d origin(1, 1);
  Eigen::Isometry2d B_in_A = carma_wm::geometry::build2dEigenTransform(origin, frame_rot);

  Eigen::Rotation2Dd new_rot(B_in_A.rotation());

  ASSERT_EQ(2, B_in_A.translation().size());
  ASSERT_NEAR(1.0, B_in_A.translation()[0], 0.000000001);
  ASSERT_NEAR(1.0, B_in_A.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI_2, new_rot.smallestAngle(), 0.000000001);

  lanelet::BasicPoint2d p_in_B(0.5, -1);
  lanelet::BasicPoint2d p_in_A = B_in_A * p_in_B;

  ASSERT_NEAR(2.0, p_in_A.x(), 0.000000001);
  ASSERT_NEAR(1.5, p_in_A.y(), 0.000000001);

  Eigen::Rotation2Dd zero_rot(0.0);
  Eigen::Isometry2d P_in_B_as_tf = carma_wm::geometry::build2dEigenTransform(p_in_B, zero_rot);
  Eigen::Isometry2d P_in_A = B_in_A * P_in_B_as_tf;
  Eigen::Rotation2Dd P_in_A_rot(P_in_A.rotation());

  ASSERT_EQ(2, P_in_A.translation().size());
  ASSERT_NEAR(2.0, P_in_A.translation()[0], 0.000000001);
  ASSERT_NEAR(1.5, P_in_A.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI_2, P_in_A_rot.smallestAngle(), 0.000000001);
}

TEST(InLaneCruisingPluginTest, curvePointInMapTF)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  Eigen::Rotation2Dd frame_rot(M_PI_2);
  lanelet::BasicPoint2d origin(1, 1);
  Eigen::Isometry2d C_in_M = carma_wm::geometry::build2dEigenTransform(origin, frame_rot);

  lanelet::BasicPoint2d p_in_B(0.5, -1);
  Eigen::Isometry2d P_in_M = plugin.curvePointInMapTF(C_in_M, p_in_B, M_PI_2);

  Eigen::Rotation2Dd P_in_M_rot(P_in_M.rotation());

  ASSERT_EQ(2, P_in_M.translation().size());
  ASSERT_NEAR(2.0, P_in_M.translation()[0], 0.000000001);
  ASSERT_NEAR(1.5, P_in_M.translation()[1], 0.000000001);
  ASSERT_NEAR(M_PI, fabs(P_in_M_rot.smallestAngle()), 0.000000001);
}

TEST(InLaneCruisingPluginTest, trajectory_from_points_times_orientations)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  lanelet::BasicPoint2d p1(0.0, 0.0);
  lanelet::BasicPoint2d p2(2.0, 0.0);
  lanelet::BasicPoint2d p3(4.5, 0.0);
  lanelet::BasicPoint2d p4(7.0, 3.0);

  std::vector<lanelet::BasicPoint2d> points = { p1, p2, p3, p4 };

  std::vector<double> times = { 0, 2, 4, 8 };
  std::vector<double> yaws = { 0.2, 0.5, 0.6, 1.0 };
  ros::Time startTime(1.0);
  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
      plugin.trajectory_from_points_times_orientations(points, times, yaws, startTime);

  ASSERT_EQ(4, traj_points.size());
  ASSERT_NEAR(1.0, traj_points[0].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(3.0, traj_points[1].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(5.0, traj_points[2].target_time.toSec(), 0.0000001);
  ASSERT_NEAR(9.0, traj_points[3].target_time.toSec(), 0.0000001);

  ASSERT_NEAR(0.0, traj_points[0].x, 0.0000001);
  ASSERT_NEAR(2.0, traj_points[1].x, 0.0000001);
  ASSERT_NEAR(4.5, traj_points[2].x, 0.0000001);
  ASSERT_NEAR(7.0, traj_points[3].x, 0.0000001);

  ASSERT_NEAR(0.0, traj_points[0].y, 0.0000001);
  ASSERT_NEAR(0.0, traj_points[1].y, 0.0000001);
  ASSERT_NEAR(0.0, traj_points[2].y, 0.0000001);
  ASSERT_NEAR(3.0, traj_points[3].y, 0.0000001);

  ASSERT_NEAR(0.2, traj_points[0].yaw, 0.0000001);
  ASSERT_NEAR(0.5, traj_points[1].yaw, 0.0000001);
  ASSERT_NEAR(0.6, traj_points[2].yaw, 0.0000001);
  ASSERT_NEAR(1.0, traj_points[3].yaw, 0.0000001);

  std::string controller_plugin = "default";
  ASSERT_EQ(0, traj_points[0].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[1].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[2].controller_plugin_name.compare(controller_plugin));
  ASSERT_EQ(0, traj_points[3].controller_plugin_name.compare(controller_plugin));

  std::string expected_plugin_name = "InLaneCruisingPlugin";
  ASSERT_EQ(0, traj_points[0].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[1].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[2].planner_plugin_name.compare(expected_plugin_name));
  ASSERT_EQ(0, traj_points[3].planner_plugin_name.compare(expected_plugin_name));
}

TEST(InLaneCruisingPluginTest, constrain_to_time_boundary)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  std::vector<PointSpeedPair> points;

  PointSpeedPair p;
  p.point = lanelet::BasicPoint2d(0, 0);
  p.speed = 1.0;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(5, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(6, 0);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(7, 0);
  points.push_back(p);

  std::vector<PointSpeedPair> time_bound_points = plugin.constrain_to_time_boundary(points, 5.0);

  ASSERT_EQ(6, time_bound_points.size());
  ASSERT_NEAR(0.0, time_bound_points[0].point.x(), 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[1].point.x(), 0.0000001);
  ASSERT_NEAR(2.0, time_bound_points[2].point.x(), 0.0000001);
  ASSERT_NEAR(3.0, time_bound_points[3].point.x(), 0.0000001);
  ASSERT_NEAR(4.0, time_bound_points[4].point.x(), 0.0000001);
  ASSERT_NEAR(5.0, time_bound_points[5].point.x(), 0.0000001);

  ASSERT_NEAR(0.0, time_bound_points[0].point.y(), 0.0000001);
  ASSERT_NEAR(0.0, time_bound_points[1].point.y(), 0.0000001);
  ASSERT_NEAR(0.0, time_bound_points[2].point.y(), 0.0000001);
  ASSERT_NEAR(0.0, time_bound_points[3].point.y(), 0.0000001);
  ASSERT_NEAR(0.0, time_bound_points[4].point.y(), 0.0000001);
  ASSERT_NEAR(0.0, time_bound_points[5].point.y(), 0.0000001);

  ASSERT_NEAR(1.0, time_bound_points[0].speed, 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[1].speed, 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[2].speed, 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[3].speed, 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[4].speed, 0.0000001);
  ASSERT_NEAR(1.0, time_bound_points[5].speed, 0.0000001);
}

TEST(InLaneCruisingPluginTest, getNearestPointIndex)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  std::vector<PointSpeedPair> points;

  PointSpeedPair p;
  p.point = lanelet::BasicPoint2d(0, 0);
  p.speed = 1.0;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 1);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 2);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 3);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 4);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(5, 5);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(6, 6);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(7, 7);
  points.push_back(p);

  cav_msgs::VehicleState state;
  state.X_pos_global = 3.3;
  state.Y_pos_global = 3.3;

  ASSERT_EQ(3, plugin.getNearestPointIndex(points, state));
}

TEST(InLaneCruisingPluginTest, get_lookahead_speed)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  lanelet::BasicPoint2d p1(10.0, 0.0);
  lanelet::BasicPoint2d p2(20.0, 0.0);
  lanelet::BasicPoint2d p3(30, 0.0);
  lanelet::BasicPoint2d p4(40, 0.0);

  std::vector<lanelet::BasicPoint2d> points = { p1, p2, p3, p4 };
  std::vector<double> speeds = {8, 9, 10, 11};

  std::vector<double> out;
  out = plugin.get_lookahead_speed(points, speeds, 10);
  ASSERT_EQ(4, out.size());
  ASSERT_EQ(9, out[0]);
  ASSERT_EQ(10, out[1]);
  ASSERT_EQ(11, out[2]);
  ASSERT_EQ(11, out[3]);

  // ASSERT_EQ(3, plugin.getNearestPointIndex(points, state));
}

TEST(InLaneCruisingPluginTest, get_adaptive_lookahead)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  
  ASSERT_EQ(5.0, plugin.get_adaptive_lookahead(2.0));
  ASSERT_EQ(2.0*6.0, plugin.get_adaptive_lookahead(6.0));
  ASSERT_EQ(25.0, plugin.get_adaptive_lookahead(22.0));


}

TEST(InLaneCruisingPluginTest, splitPointSpeedPairs)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  std::vector<PointSpeedPair> points;

  PointSpeedPair p;
  p.point = lanelet::BasicPoint2d(0, 1);
  p.speed = 1.0;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 2);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 3);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 4);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 5);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(5, 6);
  points.push_back(p);

  std::vector<lanelet::BasicPoint2d> basic_points;
  std::vector<double> speeds;

  plugin.splitPointSpeedPairs(points, &basic_points, &speeds);

  ASSERT_EQ(points.size(), basic_points.size());
  ASSERT_NEAR(0.0, basic_points[0].x(), 0.0000001);
  ASSERT_NEAR(1.0, basic_points[1].x(), 0.0000001);
  ASSERT_NEAR(2.0, basic_points[2].x(), 0.0000001);
  ASSERT_NEAR(3.0, basic_points[3].x(), 0.0000001);
  ASSERT_NEAR(4.0, basic_points[4].x(), 0.0000001);
  ASSERT_NEAR(5.0, basic_points[5].x(), 0.0000001);

  ASSERT_NEAR(1.0, basic_points[0].y(), 0.0000001);
  ASSERT_NEAR(2.0, basic_points[1].y(), 0.0000001);
  ASSERT_NEAR(3.0, basic_points[2].y(), 0.0000001);
  ASSERT_NEAR(4.0, basic_points[3].y(), 0.0000001);
  ASSERT_NEAR(5.0, basic_points[4].y(), 0.0000001);
  ASSERT_NEAR(6.0, basic_points[5].y(), 0.0000001);

  ASSERT_NEAR(1.0, speeds[0], 0.0000001);
  ASSERT_NEAR(1.0, speeds[1], 0.0000001);
  ASSERT_NEAR(1.0, speeds[2], 0.0000001);
  ASSERT_NEAR(1.0, speeds[3], 0.0000001);
  ASSERT_NEAR(1.0, speeds[4], 0.0000001);
  ASSERT_NEAR(1.0, speeds[5], 0.0000001);
}

TEST(InLaneCruisingPluginTest, compute_sub_curves)
{
  InLaneCruisingPluginConfig config;
  config.downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  std::vector<PointSpeedPair> points;
  double speed = 1.0;
  PointSpeedPair p;
  p.point = lanelet::BasicPoint2d(20, 30);
  p.speed = speed;
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 30);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(22, 30);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(23, 30);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(24, 30);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(24, 31);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(24, 32);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(24, 33);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(23, 33);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(22, 33);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 33);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 34);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 35);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 36);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(21, 37);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(22, 37);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(23, 37);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(24, 37);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(25, 37);
  points.push_back(p);
  p.point = lanelet::BasicPoint2d(26, 37);
  points.push_back(p);

  std::vector<PointSpeedPair> c1;
  p.point = lanelet::BasicPoint2d(0, 0);
  c1.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  c1.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  c1.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  c1.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 0);
  c1.push_back(p);

  std::vector<PointSpeedPair> c2;
  p.point = lanelet::BasicPoint2d(0, 0);
  c2.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  c2.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  c2.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  c2.push_back(p);

  std::vector<PointSpeedPair> c3;
  p.point = lanelet::BasicPoint2d(0, 0);
  c3.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  c3.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  c3.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  c3.push_back(p);

  std::vector<PointSpeedPair> c4;
  p.point = lanelet::BasicPoint2d(0, 0);
  c4.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  c4.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  c4.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  c4.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 0);
  c4.push_back(p);

  std::vector<PointSpeedPair> c5;
  p.point = lanelet::BasicPoint2d(0, 0);
  c5.push_back(p);
  p.point = lanelet::BasicPoint2d(1, 0);
  c5.push_back(p);
  p.point = lanelet::BasicPoint2d(2, 0);
  c5.push_back(p);
  p.point = lanelet::BasicPoint2d(3, 0);
  c5.push_back(p);
  p.point = lanelet::BasicPoint2d(4, 0);
  c5.push_back(p);
  // p.point = lanelet::BasicPoint2d(5, 0); // Last point is not added because compute_sub_curves always drops the last point
  // c5.push_back(p);

  std::vector<DiscreteCurve> discrete_curves = plugin.compute_sub_curves(points);
  ASSERT_EQ(5, discrete_curves.size());

  ASSERT_EQ(c1.size(), discrete_curves[0].points.size());
  for (size_t i = 0; i < discrete_curves[0].points.size(); i++)
  {
    auto p = discrete_curves[0].points[i];
    ASSERT_NEAR(c1[i].point.x(), p.point.x(), 0.0000001);
    ASSERT_NEAR(c1[i].point.y(), p.point.y(), 0.0000001);
    ASSERT_NEAR(c1[i].speed, p.speed, 0.0000001);
  }

  ASSERT_EQ(c2.size(), discrete_curves[1].points.size());
  for (size_t i = 0; i < discrete_curves[1].points.size(); i++)
  {
    auto p = discrete_curves[1].points[i];
    ASSERT_NEAR(c2[i].point.x(), p.point.x(), 0.0000001);
    ASSERT_NEAR(c2[i].point.y(), p.point.y(), 0.0000001);
    ASSERT_NEAR(c2[i].speed, p.speed, 0.0000001);
  }

  ASSERT_EQ(c3.size(), discrete_curves[2].points.size());
  for (size_t i = 0; i < discrete_curves[2].points.size(); i++)
  {
    auto p = discrete_curves[2].points[i];
    ASSERT_NEAR(c3[i].point.x(), p.point.x(), 0.0000001);
    ASSERT_NEAR(c3[i].point.y(), p.point.y(), 0.0000001);
    ASSERT_NEAR(c3[i].speed, p.speed, 0.0000001);
  }

  ASSERT_EQ(c4.size(), discrete_curves[3].points.size());
  for (size_t i = 0; i < discrete_curves[3].points.size(); i++)
  {
    auto p = discrete_curves[3].points[i];
    ASSERT_NEAR(c4[i].point.x(), p.point.x(), 0.0000001);
    ASSERT_NEAR(c4[i].point.y(), p.point.y(), 0.0000001);
    ASSERT_NEAR(c4[i].speed, p.speed, 0.0000001);
  }

  ASSERT_EQ(c5.size(), discrete_curves[4].points.size());
  for (size_t i = 0; i < discrete_curves[4].points.size(); i++)
  {
    auto p = discrete_curves[4].points[i];
    ASSERT_NEAR(c5[i].point.x(), p.point.x(), 0.0000001);
    ASSERT_NEAR(c5[i].point.y(), p.point.y(), 0.0000001);
    ASSERT_NEAR(c5[i].speed, p.speed, 0.0000001);
  }
}

