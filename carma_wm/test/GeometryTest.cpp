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

#include <gmock/gmock.h>
#include <iostream>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;
using namespace lanelet::units::literals;

namespace carma_wm
{
TEST(GeometryTest, computeCurvature)
{
  // Check curvature of overlapping points
  lanelet::Point2d p1(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p2(lanelet::utils::getId(), 0, 0);
  lanelet::Point2d p3(lanelet::utils::getId(), 0, 0);

  // ASSERT_NEAR(ex, act, buffer)
  ASSERT_NEAR(0.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check flat line
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 0);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(0.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave down curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, 1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(-2.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);

  // Check concave up curve
  p1 = lanelet::Point2d(lanelet::utils::getId(), 0, 0);
  p2 = lanelet::Point2d(lanelet::utils::getId(), 1, -1);
  p3 = lanelet::Point2d(lanelet::utils::getId(), 2, 0);

  ASSERT_NEAR(2.0, geometry::computeCurvature(p1, p2, p3), 0.0000001);
}

/*!
 * \brief Test utility function to generate a lanelet of fixed curvature
 * 
 * \param center_of_curvature The center of the circle that will generate the lanelet
 * \param radius_of_curvature The radius size of the circle that describes
 * \param arc_radians The number of radians to traverse the sweep of the circle
 * \param sample_count  How many points to sample evenly spaced across the arc
 * \return A lanelet matching the input geometry description
 */
std::vector<lanelet::Point3d> 
generate_const_curvature_linestring(
  lanelet::BasicPoint2d center_of_curvature,
  double radius_of_curvature,
  double arc_radians,
  double sample_count
)
{
  std::vector<lanelet::Point3d> out;

  double x, y;
  for (int i = 0; i < sample_count; i++) {
    x = center_of_curvature[0] 
      + radius_of_curvature 
      * std::cos(arc_radians / sample_count * i);
    y = center_of_curvature[1]
      + radius_of_curvature
      * std::sin(arc_radians / sample_count * i);
    
    out.push_back(getPoint(x, y, 0.0));
  }

  return out;
}

TEST(Geometry, local_curvatures)
{
  CARMAWorldModel cmw;

  auto pl1 = getPoint(-1, 0, 0);
  auto pl2 = getPoint(-1, 1, 0);
  auto pl3 = getPoint(-1, 2, 0);
  auto pr1 = getPoint(1, 0, 0);
  auto pr2 = getPoint(1, 1, 0);
  auto pr3 = getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = getLanelet(left_1, right_1);
  std::vector<lanelet::ConstLanelet> lanelets = { lanelet::utils::toConst(ll_1) };

  ///// Compute single lanelet 0 curvature
  std::vector<double> curvatures = geometry::local_curvatures(lanelets);
  ASSERT_EQ(lanelets[0].centerline2d().size(), curvatures.size());
  ASSERT_NEAR(0.0, curvatures[0], 0.0000001);
  ASSERT_NEAR(0.0, curvatures[1], 0.0000001);
  ASSERT_NEAR(0.0, curvatures[2], 0.0000001);

  // Test single lanelet constant curvature
  lanelet::BasicPoint2d origin = getBasicPoint(0.0, 0.0);
  // Curve with centerline at radius 10, curvature 1/10
  std::vector<lanelet::Point3d> left_2 = generate_const_curvature_linestring(origin, 5, M_PI / 2.0, 20); 
  std::vector<lanelet::Point3d> right_2 = generate_const_curvature_linestring(origin, 15, M_PI / 2.0, 20);
  auto ll_2 = getLanelet(left_2, right_2);
  std::vector<lanelet::ConstLanelet> lanelets2 = { lanelet::utils::toConst(ll_2) };

  std::vector<double> curvatures2 = geometry::local_curvatures(lanelets2);
  ASSERT_EQ(lanelets2[0].centerline2d().size(), curvatures2.size());

  double total = 0;
  for (double k : curvatures2) {
    total += k;
  }
  double avg = total/curvatures2.size();
  ASSERT_NEAR(0.1, avg, 0.05);

  // Test single lanelet constant curvature 2
  // Curve with centerline at radius 20, curvature 1/20
  std::vector<lanelet::Point3d> left_3 = generate_const_curvature_linestring(origin, 10, M_PI / 2.0, 10); 
  std::vector<lanelet::Point3d> right_3 = generate_const_curvature_linestring(origin, 30, M_PI / 2.0, 10);
  auto ll_3 = getLanelet(left_3, right_3);
  //lanelet::LineString3d center_ls3(lanelet::utils::getId(), center_3);

  lanelet::LaneletMapPtr map = std::move(lanelet::utils::createMap({ ll_3 }, {}));
  lanelet::utils::overwriteLaneletsCenterline(map);

  std::vector<lanelet::ConstLanelet> lanelets3 = { lanelet::utils::toConst(ll_3) };
  std::vector<double> curvatures3 = geometry::local_curvatures(lanelets3);
  ASSERT_EQ(lanelets3[0].centerline2d().size(), curvatures3.size());

  total = 0;
  for (double k : curvatures3) {
    total += k;
  }
  avg = total/(curvatures3.size());
  ASSERT_NEAR(0.05, avg, 0.05);

  // Test single lanelet constant curvature 3
  // Curve with centerline at radius 1, curvature 1
  std::vector<lanelet::Point3d> left_4 = generate_const_curvature_linestring(origin, 0.5, M_PI / 2.0, 5); 
  std::vector<lanelet::Point3d> right_4 = generate_const_curvature_linestring(origin, 1.5, M_PI / 2.0, 5);
  auto ll_4 = getLanelet(left_4, right_4);

  lanelet::LaneletMapPtr map2 = std::move(lanelet::utils::createMap({ ll_4 }, {}));
  std::cout << "LEFT" << std::endl;
  for (auto pt : ll_4.leftBound3d()) {
    std::cout << pt << std::endl;
  }
  std::cout << "RIGHT" << std::endl;
  for (auto pt : ll_4.rightBound3d()) {
    std::cout << pt << std::endl;
  }
  std::cout << "CENTER" << std::endl;
  for (auto pt : ll_4.centerline2d()) {
    std::cout << pt << std::endl;
  }
  lanelet::utils::overwriteLaneletsCenterline(map2);
  std::cout << "AFTER" << std::endl;
  for (auto pt : ll_4.centerline2d()) {
    std::cout << pt << std::endl;
  }

  std::vector<lanelet::ConstLanelet> lanelets4 = { lanelet::utils::toConst(ll_4) };
  std::vector<double> curvatures4 = geometry::local_curvatures(lanelets4);
  ASSERT_EQ(lanelets4[0].centerline2d().size(), curvatures4.size());

  // Values calculated by hand using same method
  ASSERT_NEAR(0.501551, curvatures4[0], 0.00001);
  ASSERT_NEAR(0.746148, curvatures4[1], 0.00001);
  ASSERT_NEAR(0.987687, curvatures4[2], 0.00001);
  ASSERT_NEAR(0.746139, curvatures4[3], 0.00001);
  ASSERT_NEAR(0.501544, curvatures4[4], 0.00001);

  ///// Test exception
  lanelet::Lanelet ll_empty;
  std::vector<lanelet::ConstLanelet> lanelets_5 = { lanelet::utils::toConst(ll_empty) };
  ASSERT_THROW(geometry::local_curvatures(lanelets_5), std::invalid_argument);
}

TEST(GeometryTest, trackPos)
{

  auto pl1 = getPoint(-1, 0, 0);
  auto pl2 = getPoint(-1, 1, 0);
  auto pl3 = getPoint(-1, 2, 0);
  auto pr1 = getPoint(1, 0, 0);
  auto pr2 = getPoint(1, 1, 0);
  auto pr3 = getPoint(1, 2, 0);
  std::vector<lanelet::Point3d> left_1 = { pl1, pl2, pl3 };
  std::vector<lanelet::Point3d> right_1 = { pr1, pr2, pr3 };
  auto ll_1 = getLanelet(left_1, right_1);

  ///// Test start point
  auto p = lanelet::utils::to2D(ll_1.centerline()[0]).basicPoint();
  TrackPos result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test end point
  p = lanelet::utils::to2D(ll_1.centerline().back()).basicPoint();
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(2.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point on line
  p = getBasicPoint(0, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Test mid point off line to the right
  p = getBasicPoint(0.5, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Test mid point off line to the left
  p = getBasicPoint(-0.5, 1);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point before start
  p = getBasicPoint(-0.5, -0.5);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test point after end
  p = getBasicPoint(-0.5, 2.5);
  result = geometry::trackPos(ll_1, p);
  ASSERT_NEAR(2.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Test exception throw on empty lanelet
  lanelet::Lanelet empty_ll;
  ASSERT_THROW(geometry::trackPos(empty_ll, p);, std::invalid_argument);
}

TEST(GeometryTest, trackPos_point_segment)
{
  ///// Point at start
  TrackPos result = geometry::trackPos(getBasicPoint(0, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point at end
  result = geometry::trackPos(getBasicPoint(0, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point in middle on line
  result = geometry::trackPos(getBasicPoint(0, 0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point before start but on line
  result = geometry::trackPos(getBasicPoint(0, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point after end but on line
  result = geometry::trackPos(getBasicPoint(0, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.0, result.crosstrack, 0.000001);

  ///// Point left of start
  result = geometry::trackPos(getBasicPoint(-0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of start
  result = geometry::trackPos(getBasicPoint(0.5, 0), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(0.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of end
  result = geometry::trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of end
  result = geometry::trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of middle
  result = geometry::trackPos(getBasicPoint(-0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of middle
  result = geometry::trackPos(getBasicPoint(0.5, 1), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.0, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point right of line before start
  result = geometry::trackPos(getBasicPoint(0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line before start
  result = geometry::trackPos(getBasicPoint(-0.5, -0.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(-0.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);

  ///// Point right of line after end
  result = geometry::trackPos(getBasicPoint(0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(0.5, result.crosstrack, 0.000001);

  ///// Point left of line after end
  result = geometry::trackPos(getBasicPoint(-0.5, 1.5), getBasicPoint(0, 0), getBasicPoint(0, 1));
  ASSERT_NEAR(1.5, result.downtrack, 0.000001);
  ASSERT_NEAR(-0.5, result.crosstrack, 0.000001);
}

TEST(GeometryTest, trackPos_line_string)
{
  auto p1_ = getPoint(0, 0, 0);
  auto p2_ = getPoint(0, 1, 0);
  auto p3_ = getPoint(0, 2, 0);
  auto p1 = lanelet::utils::to2D(p1_);
  auto p2 = lanelet::utils::to2D(p2_);
  auto p3 = lanelet::utils::to2D(p3_);
  lanelet::Id ls_id = lanelet::utils::getId();
  lanelet::LineString3d ls3d(ls_id, { p1_, p2_, p3_ });
  auto ls2d = lanelet::utils::to2D(ls3d);
  auto ls = ls2d.basicLineString();

  ///// Point at start
  auto result = geometry::matchSegment(p1.basicPoint(), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point at end
  result = geometry::matchSegment(p3.basicPoint(), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on first segment
  result = geometry::matchSegment(getBasicPoint(0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point on second segment
  result = geometry::matchSegment(getBasicPoint(0, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point on mid point
  result = geometry::matchSegment(getBasicPoint(0, 1.0), ls);
  ASSERT_NEAR(1.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point just passed mid point
  result = geometry::matchSegment(getBasicPoint(0, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point before start
  result = geometry::matchSegment(getBasicPoint(0, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point just passed end
  result = geometry::matchSegment(getBasicPoint(0, 2.01), ls);
  ASSERT_NEAR(2.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of start
  result = geometry::matchSegment(getBasicPoint(0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of start
  result = geometry::matchSegment(getBasicPoint(-0.5, 0.0), ls);
  ASSERT_NEAR(0.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to right of mid segment
  result = geometry::matchSegment(getBasicPoint(0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of mid segment
  result = geometry::matchSegment(getBasicPoint(-0.5, 1.01), ls);
  ASSERT_NEAR(1.01, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of end
  result = geometry::matchSegment(getBasicPoint(0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of end
  result = geometry::matchSegment(getBasicPoint(-0.5, 2.0), ls);
  ASSERT_NEAR(2.0, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of beyond end
  result = geometry::matchSegment(getBasicPoint(0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to left of beyond end
  result = geometry::matchSegment(getBasicPoint(-0.5, 2.4), ls);
  ASSERT_NEAR(2.4, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point to right of before start
  result = geometry::matchSegment(getBasicPoint(0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to left of before start
  result = geometry::matchSegment(getBasicPoint(-0.5, -0.5), ls);
  ASSERT_NEAR(-0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of second segment
  result = geometry::matchSegment(getBasicPoint(5.5, 1.5), ls);
  ASSERT_NEAR(1.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(5.5, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Point far to right in middle of first segment
  result = geometry::matchSegment(getBasicPoint(1.0, 0.5), ls);
  ASSERT_NEAR(0.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p1.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).second);

  ///// Point to beyond end of second segment
  result = geometry::matchSegment(getBasicPoint(1.0, 2.5), ls);
  ASSERT_NEAR(2.5, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(1.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(p2.basicPoint(), std::get<1>(result).first);
  ASSERT_EQ(p3.basicPoint(), std::get<1>(result).second);

  ///// Test exception throw on empty linestring
  lanelet::LineString2d empty_ls;
  ASSERT_THROW(geometry::matchSegment(getBasicPoint(1.0, 2.5), empty_ls.basicLineString()), std::invalid_argument);

  ///// Create concave down triangle
  auto pa_ = getPoint(0, 0, 0);
  auto pb_ = getPoint(0.5, 1, 0);
  auto pc_ = getPoint(1, 0, 0);
  auto pa = lanelet::utils::to2D(pa_).basicPoint();
  auto pb = lanelet::utils::to2D(pb_).basicPoint();
  auto pc = lanelet::utils::to2D(pc_).basicPoint();
  lanelet::LineString3d ls_3d(lanelet::utils::getId(), { pa_, pb_, pc_ });
  auto ls_2d = lanelet::utils::to2D(ls_3d);
  auto ls_ = ls_2d.basicLineString();

  ///// Point before start of triangle
  result = geometry::matchSegment(getBasicPoint(-0.5, -1.0), ls_);
  ASSERT_NEAR(-1.11803398875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point above midpoint of triangle
  result = geometry::matchSegment(getBasicPoint(0.5, 1.5), ls_);
  ASSERT_NEAR(1.565247, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(-0.2236067, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle
  result = geometry::matchSegment(getBasicPoint(0.5, 0.25), ls_);
  ASSERT_NEAR(0.44721359, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.335410, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point after end of triangle
  result = geometry::matchSegment(getBasicPoint(1.5, -1.0), ls_);
  ASSERT_NEAR(3.35410196625, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.0, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);

  ///// Point below midpoint of triangle to left
  result = geometry::matchSegment(getBasicPoint(0.4, 0.25), ls_);
  ASSERT_NEAR(0.402492, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pa, std::get<1>(result).first);
  ASSERT_EQ(pb, std::get<1>(result).second);

  ///// Point below midpoint of triangle to right
  result = geometry::matchSegment(getBasicPoint(0.6, 0.25), ls_);
  ASSERT_NEAR(1.83357598875, std::get<0>(result).downtrack, 0.000001);
  ASSERT_NEAR(0.245967, std::get<0>(result).crosstrack, 0.000001);
  ASSERT_EQ(pb, std::get<1>(result).first);
  ASSERT_EQ(pc, std::get<1>(result).second);
}

TEST(Geometry, objectToMapPolygon)
{
  geometry_msgs::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  lanelet::BasicPolygon2d result = geometry::objectToMapPolygon(pose, size);

  ASSERT_NEAR(result[0][0], 5.0, 0.00001);
  ASSERT_NEAR(result[0][1], 7.0, 0.00001);

  ASSERT_NEAR(result[1][0], 7.0, 0.00001);
  ASSERT_NEAR(result[1][1], 7.0, 0.00001);

  ASSERT_NEAR(result[2][0], 7.0, 0.00001);
  ASSERT_NEAR(result[2][1], 3.0, 0.00001);

  ASSERT_NEAR(result[3][0], 5.0, 0.00001);
  ASSERT_NEAR(result[3][1], 3.0, 0.00001);
}

void rpyFromQuatMsg(const geometry_msgs::Quaternion& q_msg, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat;
  tf2::convert(q_msg, quat);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);
}

TEST(GeometryTest, compute_tangent_orientations_straight)
{
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 25);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(25_mph, wm);

  /**
   *
   *
   *        |1203|1213|1223|
   *        | _  _  _  _  _|
   *        |1202| Ob |1222|
   *        | _  _  _  _  _|
   *        |1201|1211|1221|    num   = lanelet id hardcoded for easier testing
   *        | _  _  _  _  _|    |     = lane lines
   *        |1200|1210|1220|    - - - = Lanelet boundary
   *        |              |    O     = Default Obstacle
   *        ****************
   *           START_LINE
   */

  carma_wm::test::setRouteByIds({ 1200, 1201, 1202, 1203 }, wm);

  auto route_lanelets = wm->getRoute()->shortestPath();

  lanelet::ConstLanelets lanelets_as_vec;

  for (lanelet::ConstLanelet ll : route_lanelets)
  {
    lanelets_as_vec.push_back(ll);
  }

  std::vector<double> result;
  lanelet::BasicLineString2d centerline = carma_wm::geometry::concatenate_lanelets(lanelets_as_vec);  
  result = carma_wm::geometry::compute_tangent_orientations(centerline);
  ASSERT_EQ(9, result.size());

  for (double yaw : result)
  {
    ASSERT_NEAR(M_PI_2, yaw, 0.000001);
  }
}

TEST(GeometryTest, concatenate_line_string_dedupe) 
{
  auto p1 = Eigen::Vector2d(-1, 0);
  auto p2 = Eigen::Vector2d(-1, 1);
  auto p3 = Eigen::Vector2d(-1, 2);
  auto p4 = Eigen::Vector2d(-1, 2.001);
  auto p5 = Eigen::Vector2d(-1, 3);
  auto p6 = Eigen::Vector2d(-1, 4);
  const lanelet::BasicLineString2d line1 = { p1, p2, p3 };
  const lanelet::BasicLineString2d line2 = { p4, p4, p6 };

  auto result = carma_wm::geometry::concatenate_line_strings(line1, line2);

  ASSERT_EQ(5, result.size());

  p1 = Eigen::Vector2d(-1, 0);
  p2 = Eigen::Vector2d(-1, 1);
  p3 = Eigen::Vector2d(-1, 2);
  p4 = Eigen::Vector2d(-1, 2.5);
  p5 = Eigen::Vector2d(-1, 3);
  p6 = Eigen::Vector2d(-1, 4);
  const lanelet::BasicLineString2d line3 = { p1, p2, p3 };
  const lanelet::BasicLineString2d line4 = { p4, p4, p6 };

  result = carma_wm::geometry::concatenate_line_strings(line3, line4);

  ASSERT_EQ(6, result.size());
}

TEST(GeometryTest, compute_tangent_orientations_curved)
{

  // This tests creates a 90 deg left turn from x = 0 that is followed by a short straight away
  // The inner turn radius is 30m and the outer radius is 34m
  // The straight away is 4m long. The turn is one lanelet the straight section is another
  std::shared_ptr<CARMAWorldModel> wm = std::make_shared<CARMAWorldModel>();

  int segments = 6;
  double rad_increment = M_PI_2 / (double) segments;
  double inner_radius = 30;
  double outer_radius = 34;
  std::vector<lanelet::Point3d> left_points_1, right_points_1, left_points_2, right_points_2;
  double angle = 0;
  for (int i = 0; i <= segments; i++) {
    left_points_1.push_back(carma_wm::test::getPoint(inner_radius * cos(angle), inner_radius * sin(angle), 0));
    right_points_1.push_back(carma_wm::test::getPoint(outer_radius * cos(angle), outer_radius * sin(angle), 0));
    angle += rad_increment;
  }

  left_points_2.push_back(left_points_1.back());
  left_points_2.push_back(carma_wm::test::getPoint(-4, 30, 0));

  right_points_2.push_back(right_points_1.back());
  right_points_2.push_back(carma_wm::test::getPoint(-4, 34, 0));

  lanelet::Lanelet ll_1 = carma_wm::test::getLanelet(left_points_1, right_points_1);
  lanelet::Lanelet ll_2 = carma_wm::test::getLanelet(left_points_2, right_points_2);

  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
  lanelet::MapConformer::ensureCompliance(map);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(25_mph, wm);

  carma_wm::test::setRouteByLanelets({ ll_1, ll_2 }, wm);

  auto route_lanelets = wm->getRoute()->shortestPath();


  std::vector<double> result;
  lanelet::BasicLineString2d centerline = carma_wm::geometry::concatenate_lanelets({ lanelet::traits::toConst(ll_1), lanelet::traits::toConst(ll_2) });
  result = carma_wm::geometry::compute_tangent_orientations(centerline);

  ASSERT_EQ(15, result.size());

  ASSERT_NEAR(1.7017, result[0], 0.00001); // First point has some error which is allowable due to mathemtical constraints on calculating the tangent
  ASSERT_NEAR(1.7017, result[1], 0.00001);
  ASSERT_NEAR(1.82437, result[2], 0.00001);
  ASSERT_NEAR(1.9635, result[3], 0.00001);
  ASSERT_NEAR(2.0944, result[4], 0.00001);
  ASSERT_NEAR(2.22529, result[5], 0.00001);
  ASSERT_NEAR(2.35619, result[6], 0.00001);
  ASSERT_NEAR(2.48709, result[7], 0.00001);
  ASSERT_NEAR(2.60977, result[8], 0.00001);
  ASSERT_NEAR(2.74889, result[9], 0.00001);
  ASSERT_NEAR(2.87157, result[10], 0.00001);
  ASSERT_NEAR(3.01069, result[11], 0.00001);
  ASSERT_NEAR(3.05132, result[12], 0.00001);
  ASSERT_NEAR(3.14159, result[13], 0.00001);
  ASSERT_NEAR(3.14159, result[14], 0.00001);

  // Verify empty centerline gives 0 output
  lanelet::BasicLineString2d empty_ls;
  result = carma_wm::geometry::compute_tangent_orientations(empty_ls);
  ASSERT_EQ(0, result.size());

}

TEST(GeometryTest, point_to_point_yaw)
{
    lanelet::BasicPoint2d point1{1.0, 1.0};
    lanelet::BasicPoint2d point2{1.0, 2.0};
    double res = geometry::point_to_point_yaw(point1, point2);
    EXPECT_NEAR(1.57, res, 0.1);
}

TEST(GeometryTest, circular_arc_curvature)
{
    lanelet::BasicPoint2d point1{1.0, 1.0};
    lanelet::BasicPoint2d point2{3.0, 4.0};
    double res = geometry::circular_arc_curvature(point1, point2);
    EXPECT_NEAR(0.46153846, res, 0.1);
}


}  // namespace carma_wm
