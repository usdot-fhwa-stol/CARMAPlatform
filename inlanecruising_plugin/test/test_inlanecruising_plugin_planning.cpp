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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm/MapConformer.h>
#include <carma_wm/CARMAWorldModel.h>
#include <ros/console.h>
#include <unsupported/Eigen/Splines>
#include <carma_utils/containers/containers.h>
#include <tf/LinearMath/Vector3.h>


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
#include <unordered_set>


typedef Eigen::Spline<double, 2> Spline2d;

using namespace lanelet::units::literals;
namespace inlanecruising_plugin
{


TEST(InLaneCruisingPluginTest, testPlanningCallback)
{
  InLaneCruisingPluginConfig config;
  config.default_downsample_ratio = 1;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  InLaneCruisingPlugin plugin(wm, config, [&](auto msg) {});

  auto map = carma_wm::test::buildGuidanceTestMap(3.7, 10);

  wm->setMap(map);
  carma_wm::test::setSpeedLimit(15_mph, wm);

  /**
   * Total route length should be 100m
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

  cav_srvs::PlanTrajectoryRequest req;
  req.vehicle_state.X_pos_global = 1.5;
  req.vehicle_state.Y_pos_global = 5;
  req.vehicle_state.orientation = 0;
  req.vehicle_state.longitudinal_vel = 0.0;

  cav_msgs::Maneuver maneuver;
  maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.lane_id = 1200;
  maneuver.lane_following_maneuver.start_dist = 5.0;
  maneuver.lane_following_maneuver.start_time = ros::Time(0.0);
  maneuver.lane_following_maneuver.start_speed = 0.0;

  maneuver.lane_following_maneuver.end_dist = 14.98835712;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = ros::Time(4.4704);

  cav_msgs::Maneuver maneuver2;
  maneuver2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.lane_id = 1200;
  maneuver2.lane_following_maneuver.start_dist = 14.98835712;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = ros::Time(4.4704);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = ros::Time(4.4704 + 7.45645430685);

  // Create a third maneuver of a different type to test the final element in resp.related_maneuvers
  cav_msgs::Maneuver maneuver3;
  maneuver3.type = cav_msgs::Maneuver::LANE_CHANGE;

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);
  req.maneuver_plan.maneuvers.push_back(maneuver3);

  req.maneuver_index_to_plan = 0;

  cav_srvs::PlanTrajectoryResponse resp;

  plugin.plan_trajectory_cb(req, resp);

  EXPECT_EQ(1, resp.related_maneuvers.back());

}

/*
Using this file:
    1) Set the file path to your OSM file
    2) Set the route_ids to the route you wish to test

    NOTE: The test is disabled by default. Enable it by removing the DISABLED_ prefix from the test name
*/

TEST(WaypointGeneratorTest, DISABLED_test_full_generation)
{
  
 int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 111, 110, 113, 135, 138 };

  // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  lanelet::MapConformer::ensureCompliance(map, 80_mph);

  InLaneCruisingPluginConfig config;
  config.lateral_accel_limit = 1.5;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  wm->setMap(map);

  InLaneCruisingPlugin inlc(wm, config, [&](auto msg) {});
  
  auto routing_graph = wm->getMapRoutingGraph();

  // Output graph for debugging
  //routing_graph->exportGraphViz("/workspaces/carma_ws/carma/src/carma-platform/waypoint_generator/routing.viz");

  carma_wm::test::setRouteByIds(route_ids, wm);

  auto p = wm->getMap()->laneletLayer.get(130).centerline()[3];
  ROS_WARN_STREAM("Start Point: " << p.x() << ", " << p.y());

  // -159.666, 521.683

  cav_srvs::PlanTrajectoryRequest req;

  req.vehicle_state.X_pos_global = -107;
  req.vehicle_state.Y_pos_global = 311.904;
  req.vehicle_state.orientation = -2.7570977;
  req.vehicle_state.longitudinal_vel = 0.0;

  cav_msgs::Maneuver maneuver;
  maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver.lane_following_maneuver.lane_id = 110;
  maneuver.lane_following_maneuver.start_dist = 14.98835712 + 45+ 180;
  maneuver.lane_following_maneuver.start_time = ros::Time(0.0);
  maneuver.lane_following_maneuver.start_speed = 0.0;

  maneuver.lane_following_maneuver.end_dist = 14.98835712 + 50.0 + 45 + 200;
  maneuver.lane_following_maneuver.end_speed = 6.7056;
  maneuver.lane_following_maneuver.end_time = ros::Time(8);

  cav_msgs::Maneuver maneuver2;
  maneuver2.type = cav_msgs::Maneuver::LANE_FOLLOWING;
  maneuver2.lane_following_maneuver.lane_id = 110;
  maneuver2.lane_following_maneuver.start_dist = 14.98835712 + 45+ 202;
  maneuver2.lane_following_maneuver.start_speed = 6.7056;
  maneuver2.lane_following_maneuver.start_time = ros::Time(4.4704);

  maneuver2.lane_following_maneuver.end_dist = 14.98835712 + 50.0 + 45 + 250;
  maneuver2.lane_following_maneuver.end_speed = 6.7056;
  maneuver2.lane_following_maneuver.end_time = ros::Time(4.4704 + 7.45645430685 + 37.31);

  req.maneuver_plan.maneuvers.push_back(maneuver);
  req.maneuver_plan.maneuvers.push_back(maneuver2);

  cav_srvs::PlanTrajectoryResponse resp;

  inlc.plan_trajectory_cb(req, resp);


}

TEST(WaypointGeneratorTest, DISABLED_test_compute_fit_full_generation)
{
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;

  // File location of osm file
  std::string file = "/workspaces/carma/AOI_1_TFHRC_faster_pretty.osm";    
  // The route ids that will form the route used
  std::vector<lanelet::Id> route_ids = { 130, 129 };

  // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  lanelet::MapConformer::ensureCompliance(map, 80_mph);

  InLaneCruisingPluginConfig config;
  config.lateral_accel_limit = 1.5;
  std::shared_ptr<carma_wm::CARMAWorldModel> wm = std::make_shared<carma_wm::CARMAWorldModel>();
  wm->setMap(map);

  InLaneCruisingPlugin inlc(wm, config, [&](auto msg) {});
  
  auto routing_graph = wm->getMapRoutingGraph();

  carma_wm::test::setRouteByIds(route_ids, wm);

  auto route = wm->getRoute()->shortestPath();
  std::vector<lanelet::ConstLanelet> lanelets;
  for (auto llt : route)
  {
    lanelets.push_back(llt);
  }
  // Get centerline
  lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets);
  
  // Downsample
  std::vector<lanelet::BasicPoint2d> downsampled_points;
  downsampled_points.reserve((route_geometry.size() / config.default_downsample_ratio) + 1);

  for (int i = 0; i < route_geometry.size(); i += config.default_downsample_ratio)
  {
    downsampled_points.push_back(route_geometry[i]);
    // Uncomment to print and check if this original map matches with the generated one below 
    // ROS_INFO_STREAM("Original point: x: " << route_geometry[i].x() << "y: " << route_geometry[i].y());
  }

  std::unique_ptr<basic_autonomy::smoothing::SplineI> fit_curve = basic_autonomy:: waypoint_generation::compute_fit(downsampled_points);
  
  std::vector<lanelet::BasicPoint2d> spline_points;

  // Following logic is written for BSpline library. Switch with appropriate call of the new library if different.
  double parameter = 0.0;
  for(int i=0; i< downsampled_points.size(); i++){
    lanelet::BasicPoint2d pt = (*fit_curve)(parameter);
    // Uncomment to print and check if this generated map matches with the original one above 
    // ROS_INFO_STREAM("BSpline point: x: " << values.x() << "y: " << values.y());
    spline_points.push_back(pt);
    parameter += 1.0/(downsampled_points.size()*1.0);
  }

  ASSERT_EQ(spline_points.size(), downsampled_points.size());
  int error_count = 0;
  
  for(int i = 1; i < downsampled_points.size(); i ++)
  {
    // tag as erroneous if directions of generated points are not within 5 degree of those of original points
    tf::Vector3 original_vector(downsampled_points[i].x() - downsampled_points[i-1].x(), 
                      downsampled_points[i].y() - downsampled_points[i-1].y(), 0);
    original_vector.setZ(0);
    tf::Vector3 spline_vector(spline_points[i].x() - spline_points[i-1].x(), 
                      spline_points[i].y() - spline_points[i-1].y(), 0);
    spline_vector.setZ(0);
    double angle_in_rad = std::fabs(tf::tfAngle(original_vector, spline_vector));
    if (angle_in_rad > 0.08)  error_count ++;
  }

  // We say it is passing if there is less than 10% error in total number of points
  ROS_INFO_STREAM("Total points above 5 degree difference in their direction:" << error_count 
    << ", which is " << (double)error_count/(double)downsampled_points.size()*100 << "% of total");
  ASSERT_TRUE((double)error_count/(double)downsampled_points.size() < 0.1); 
}

};  // namespace inlanecruising_plugin


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();
    ROSCONSOLE_AUTOINIT;
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
    return RUN_ALL_TESTS();
}