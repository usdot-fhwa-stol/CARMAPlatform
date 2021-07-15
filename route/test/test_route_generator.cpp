/*
 * Copyright (C) 2020 LEIDOS.
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
#include <carma_wm/WMTestLibForGuidance.h>
#include "route_generator_worker.h"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/Traits.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include "route_state_worker.h"
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

/*
Using this file:
    1) Libraries you should have: carma-base, carma-config, carma-msgs, 
        carma-platform, carma-utils, caram-web-ui, opendrive2lanelet.
    2) Then the test can be built with the command: 
        ./carma_build -c /workspaces/carma_ws/carma/ -a /workspaces/carma_ws/autoware.ai/ -x -m "--only-pkg-with-deps route"
    3) Update the osm file location and starting/ending IDs to match the file you want to test
    4) Run the unit test with
        catkin_make run_tests_route
    5) Confirm that the test passed and that the list of lanelet IDs does traverse from the start to the end
*/


TEST(RouteGeneratorTest, testRouteVisualizerCenterLineParser)
{
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../resource/map/vector_map.osm";    
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 1346;
    lanelet::Id end_id = 1351;

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try {
        start_lanelet = map->laneletLayer.get(start_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    
    // Create MarkerArray to test

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE_LIST;//
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "route_visualizer";

    marker.scale.x = 0.65;
    marker.scale.y = 0.65;
    marker.scale.z = 0.65;
    marker.frame_locked = true;

    marker.id = 0;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    geometry_msgs::Point p1;
    p1.x = start_lanelet.centerline3d().front().x();
    p1.y = start_lanelet.centerline3d().front().y();

    marker.points.push_back(p1);

    geometry_msgs::Point p2;
    p2.x = start_lanelet.centerline3d().back().x();
    p2.y = start_lanelet.centerline3d().back().y();

    marker.points.push_back(p2);


    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        auto test_msg = worker.compose_route_marker_msg(route);
        ASSERT_EQ(marker.points.size(), test_msg.points.size());
        EXPECT_NEAR(marker.points[0].x, test_msg.points[0].x, 10.0);
        EXPECT_NEAR(marker.points[0].y, test_msg.points[0].y, 10.0);
        EXPECT_NEAR(marker.points[1].x, test_msg.points[1].x, 10.0);
        EXPECT_NEAR(marker.points[1].y, test_msg.points[1].y, 10.0);
    }
    
    
}

TEST(RouteGeneratorTest, testLaneletRoutingVectorMap)
{
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../resource/map/vector_map.osm";    
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 1346;
    lanelet::Id end_id = 1349;

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try {
        start_lanelet = map->laneletLayer.get(start_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    // Output graph for debugging
    map_graph->exportGraphViz("../routing.txt");

    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        cav_msgs::Route route_msg_ = worker.compose_route_msg(route);


        ASSERT_TRUE(route_msg_.shortest_path_lanelet_ids.size() > 0);
        ASSERT_TRUE(route_msg_.route_path_lanelet_ids.size() > 0);
    }
}

TEST(RouteGeneratorTest, testLaneletRoutingTown02VectorMap)
{
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // If the output is an error about the geoReference field in the osm file, here is a correct example. If the lat/lon coordinates are already correct, simply add:
    // <geoReference>+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +geoidgrids=egm96_15.gtx +vunits=m +no_defs</geoReference>

    // File location of osm file
    std::string file = "../resource/map/town01_vector_map_1.osm";
    // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;
    /***
     * VAVLID PATHs (consists of lanenet ids): (This is also the shortest path because certain Lanelets missing)
     * 159->160->164->136->135->137->144->121; 
     * 159->160->164->136->135->137->144->118;
     * 168->170->111
     * 159->161->168->170->111
     * 167->169->168->170->111
     * 115->146->140->139->143->167->169->168->170->111 
     * 141->139->143->167->169->168->170->111 
     * 127->146->140->139->143->167->169->168->170->111 
     * 101->100->104->167->169->168->170->111 (a counter cLock circle) 
     * **/

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;

    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
    // Output graph for debugging
    map_graph->exportGraphViz("../routing2.txt");

    // Computes the shortest path and prints the list of lanelet IDs to get from the start to the end. Can be manually confirmed in JOSM
    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    if(!route) {
        ASSERT_FALSE(true);
        std::cout << "Route not generated." << " ";
    } else {
        std::cout << "shortest path: \n";
        for(const auto& ll : route.get().shortestPath()) {
            std::cout << ll.id() << " ";
        }
        std::cout << "\n";
        cav_msgs::Route route_msg_ = worker.compose_route_msg(route);
        ASSERT_TRUE(route_msg_.shortest_path_lanelet_ids.size() > 0);
        ASSERT_TRUE(route_msg_.route_path_lanelet_ids.size() > 0);
    }

}

TEST(RouteGeneratorTest, test_crosstrack_error_check)
{
     std::shared_ptr<carma_wm::WMListener> wml;
    std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    geometry_msgs::PoseStamped msg;

    //Create route msg
    cav_msgs::Route route_msg;

  // File location of osm file
    std::string file = "../resource/map/town01_vector_map_1.osm";
     // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;

    //Load map parameters

    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
    cmw->carma_wm::CARMAWorldModel::setMap(map);

     // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;


    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }


    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    route_msg = worker.compose_route_msg(route);
    ASSERT_TRUE(route_msg.route_path_lanelet_ids.size() > 0);


    /*Test 1: Vehicle is out of bounds*/

    //Assign vehicle position
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;

    worker.setWorldModelPtr(cmw);
    worker.set_CTE_count_max(0);
    worker.set_CTE_dist(1.0);

    geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));

    /*Compare vehicle position to the route bounds */
    lanelet::BasicPoint2d current_loc(mpt->pose.position.x, mpt->pose.position.y);

    auto current_lanelet = worker.get_closest_lanelet_from_route_llts(current_loc);

    worker.pose_cb(mpt);

    lanelet::BasicPoint2d position;
    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), false);
    ASSERT_EQ(worker.crosstrack_error_check(mpt, start_lanelet), true); //The vehicle will show crosstrack error, so the value should return true

    /*Test 2: Vehicle is in bounds, no crosstrack error*/

    //Use position values to show the case when there is no crosstrack error
    worker.set_CTE_dist(1.0);

    //Assign vehicle position
    msg.pose.position.x = -9.45542;
    msg.pose.position.y = -182.324;

    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;

    geometry_msgs::PoseStampedPtr mpt2(new geometry_msgs::PoseStamped(msg));

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), true);
    ASSERT_EQ(worker.crosstrack_error_check(mpt2, start_lanelet), false);

    /*Test 3: Vehicle is out of bounds, and has exceeded the maximum number of consecutive timesteps outside of the route allowable before triggering LEFT_ROUTE*/
    worker.set_CTE_count_max(1);
    worker.set_CTE_dist(1.0);

    position.x()= 0.0;
    position.y()= 0.0;

    ASSERT_EQ(boost::geometry::within(position, start_lanelet.polygon2d()), false);
    ASSERT_EQ(worker.crosstrack_error_check(mpt, start_lanelet), false); //The vehicle will show no crosstrack error, so the value should return false
    ASSERT_EQ(worker.crosstrack_error_check(mpt, start_lanelet), true); //The vehicle will show crosstrack error, so the value should return true



}

TEST(RouteGeneratorTest, test_set_active_route_cb)
{
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker;
    worker.set_route_file_path("../resource/route/");
    cav_srvs::GetAvailableRoutesRequest req;
    cav_srvs::GetAvailableRoutesResponse resp;
    ASSERT_TRUE(worker.get_available_route_cb(req, resp));

    std::string proj = "+proj=tmerc +lat_0=38.95197911150576 +lon_0=-77.14835128349988 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
    
    std_msgs::String str_msg;
    str_msg.data = proj;
    std_msgs::StringConstPtr msg_ptr(new std_msgs::String(str_msg));
    worker.georeference_cb(msg_ptr);  // Set projection

    std::cout << "Available Route : " << resp.availableRoutes.size() << "\n";
    ASSERT_EQ(4, resp.availableRoutes.size());
    for(auto i = 0; i < resp.availableRoutes.size();i++)    
    {
        if(resp.availableRoutes[i].route_id  == "tfhrc_test_route")
        {
            std::cout <<"C-HUB : " << resp.availableRoutes[i].route_name << "\n";
            auto gps_points = worker.load_route_destination_gps_points_from_route_id("tfhrc_test_route");
            auto map_points = worker.load_route_destinations_in_map_frame(gps_points);
            std::cout << "Point Size : " << map_points.size()<<"\n";
            ASSERT_EQ(5, map_points.size());
            ASSERT_NEAR(-170.21389078, map_points[0].x(), 0.001);
            ASSERT_NEAR(498.11200005, map_points[0].y(), 0.001);  
            ASSERT_NEAR(72, map_points[0].z(), 0.001);
        }
    }
    cav_srvs::SetActiveRouteRequest req2;
    cav_srvs::SetActiveRouteResponse resp2;
    geometry_msgs::PoseStamped msg;

    //Assign vehicle position
    msg.pose.position.x = 1106580;
    msg.pose.position.y = 894697;
    geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));
    worker.pose_cb(mpt);

    resp2.errorStatus = 0;

    for(auto i: resp.availableRoutes)
    {
        if(i.route_id  == "tfhrc_test_route")
        {
            req2.routeID = i.route_id;
            req2.choice = cav_srvs::SetActiveRouteRequest::ROUTE_ID;

            ASSERT_EQ(worker.set_active_route_cb(req2, resp2), true);
        }

   }
}

TEST(RouteGeneratorTest, test_duplicate_lanelets_in_shortest_path)
{
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    // File location of the osm file for this test case:
    std::string file = "../resource/map/town01_vector_map_1.osm";

    // The parsing in this file was copied from https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/carma_wm_ctrl/test/MapToolsTest.cpp
    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

    // Starting and ending lanelet IDs (these were obtained from viewing the osm file in JOSM)
    lanelet::Id start_id = 111;
    lanelet::Id end_id = 111;
    lanelet::Lanelet start_lanelet = map->laneletLayer.get(start_id);
    lanelet::Lanelet end_lanelet = map->laneletLayer.get(end_id);

    // Create and populate the 'via_lanelets_vector':
    lanelet::ConstLanelets via_lanelets_vector;
    lanelet::Id middle_id = 167;
    via_lanelets_vector.push_back(map->laneletLayer.get(middle_id));

    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    auto route_with_duplicates = map_graph->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);

    // Test that the shortest path in 'route_with_duplicates' contains duplicate lanelet IDs
    // The shortest path is 111 -> 101 -> 100 -> 104 -> 167 -> 169 -> 168 -> 170 -> 111
    ASSERT_EQ(worker.check_for_duplicate_lanelets_in_shortest_path(route_with_duplicates.get()), true);

    // Change the ending lanelet ID and update the route so the shortest path does not contain duplicate lanelet IDs
    end_id = 170;
    end_lanelet = map->laneletLayer.get(end_id);
    auto route_without_duplicates = map_graph->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);

    // Test that the shortest path in 'route_without_duplicates' does not contain duplicate Lanelet IDs
    // The shortest path is 111 -> 101 -> 100 -> 104 -> 167 -> 169 -> 168 -> 170
    ASSERT_EQ(worker.check_for_duplicate_lanelets_in_shortest_path(route_without_duplicates.get()), false);
}

TEST(RouteGeneratorTest, test_reroute_after_route_invalidation)
{
    route::RouteGeneratorWorker worker;

    auto cmw= carma_wm::test::getGuidanceTestMap();
    worker.setWorldModelPtr(cmw);

   // set route here
    carma_wm::test::setRouteByIds({1200, 1201,1202,1203}, cmw);

    lanelet::BasicPoint2d end_point{1.85, 87.5};

    std::vector<lanelet::BasicPoint2d> dest_points;
    dest_points.push_back(end_point);

    geometry_msgs::PoseStamped msg;
    //Assign vehicle position
    msg.pose.position.x = 1.85;
    msg.pose.position.y = 0.1;

    geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));

    worker.pose_cb(mpt);

    auto route = worker.reroute_after_route_invalidation(dest_points);

    ASSERT_EQ(dest_points.size(), 1);
    ASSERT_TRUE(!!route);
    ASSERT_EQ(route->shortestPath().size(), 4);

}

TEST(RouteGeneratorTest, test_setReroutingChecker)
{
    route::RouteGeneratorWorker worker;
    bool flag = false;
    worker.setReroutingChecker([&]{
        flag = true;
        return flag;
    });
    EXPECT_NO_THROW(worker.reroutingChecker());
    ASSERT_EQ(true,worker.reroutingChecker());
}

TEST(RouteGeneratorTest, test_get_closest_lanelet_from_route_llts)
{
     std::shared_ptr<carma_wm::WMListener> wml;
    std::shared_ptr<carma_wm::CARMAWorldModel> cmw=std::make_shared<carma_wm::CARMAWorldModel>();
    route::RouteGeneratorWorker worker;

    int projector_type = 0;
    std::string target_frame;
    lanelet::ErrorMessages load_errors;

    geometry_msgs::PoseStamped msg;

    //Create route msg
    cav_msgs::Route route_msg;

  // File location of osm file
    std::string file = "../resource/map/town01_vector_map_1.osm";
     // Starting and ending lanelet IDs. It's easiest to grab these from JOSM
    lanelet::Id start_id = 101;
    lanelet::Id end_id = 111;

    //Load map parameters

    lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);
    lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
    lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);
    cmw->carma_wm::CARMAWorldModel::setMap(map);

     // Grabs lanelet elements from the start and end IDs. Fails the unit test if there is no lanelet with the matching ID
    lanelet::Lanelet start_lanelet;
    lanelet::Lanelet end_lanelet;


    try 
    {
        //get lanelet layer
        start_lanelet = map->laneletLayer.get(start_id);        
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified starting lanelet Id of " << start_id << " does not exist in the provided map.";
    }
    try {
        end_lanelet = map->laneletLayer.get(end_id);
    }
    catch (const lanelet::NoSuchPrimitiveError& e) {
        FAIL() << "The specified ending lanelet Id of " << end_id << " does not exist in the provided map.";
    }


    lanelet::LaneletMapConstPtr const_map(map);
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::VehicleCar);
    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    const auto route = map_graph->getRoute(start_lanelet, end_lanelet);
    route_msg = worker.compose_route_msg(route);
    ASSERT_TRUE(route_msg.route_path_lanelet_ids.size() > 0);

    worker.setWorldModelPtr(cmw);
    worker.set_CTE_count_max(0);
    worker.set_CTE_dist(1.0);

     //Assign vehicle position
    msg.pose.position.x = -9.45542;
    msg.pose.position.y = -182.324;

    lanelet::BasicPoint2d position;
    position.x()= msg.pose.position.x;
    position.y()= msg.pose.position.y;

    geometry_msgs::PoseStampedPtr mpt(new geometry_msgs::PoseStamped(msg));

    worker.pose_cb(mpt);
    worker.addllt(start_lanelet);
    lanelet::ConstLanelet llt = worker.get_closest_lanelet_from_route_llts(position);

    ASSERT_EQ(worker.crosstrack_error_check(mpt, llt), false);
    ASSERT_EQ(boost::geometry::within(position, llt.polygon2d()), true);



}



// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
