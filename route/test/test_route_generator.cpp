/*
 * Copyright (C) 2020-2021 LEIDOS.
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
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);

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
    visualization_msgs::MarkerArray route_marker_msg;
    route_marker_msg.markers={};

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::SPHERE;//
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "route_visualizer";

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.frame_locked = true;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.pose.position.x = start_lanelet.centerline3d().front().x();
    marker.pose.position.y = start_lanelet.centerline3d().front().y();

    route_marker_msg.markers.push_back(marker);

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
        EXPECT_EQ(route_marker_msg.markers[0].pose.position.x, test_msg.markers[0].pose.position.x);
        EXPECT_EQ(route_marker_msg.markers[0].pose.position.y, test_msg.markers[0].pose.position.y);
        EXPECT_EQ(route_marker_msg.markers[1].pose.position.x, test_msg.markers[1].pose.position.x);
        EXPECT_EQ(route_marker_msg.markers[1].pose.position.y, test_msg.markers[1].pose.position.y);
    }
}



TEST(RouteGeneratorTest, testLaneletRoutingVectorMap)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);

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
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);

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

TEST(RouteGeneratorTest, testReadLanelet111RouteFile)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);
    worker.set_route_file_path("../resource/route/");
    cav_srvs::GetAvailableRoutesRequest req;
    cav_srvs::GetAvailableRoutesResponse resp;
    ASSERT_TRUE(worker.get_available_route_cb(req, resp));
    for(auto i = 0; i < resp.availableRoutes.size();i++)    
    {
        std::cout<< "Route #: " << (i+1) << "\n";
        std::cout<< "Route ID: " << resp.availableRoutes[i].route_id << "\n";
        std::cout<< "Route Name: " << resp.availableRoutes[i].route_name << "\n";
    }
    auto points = worker.load_route_destinations_in_ecef("Test_lanelet111_route_2");
    std::cout << "Point Size : " << points.size()<<"\n";
    ASSERT_EQ(8, points.size());    
    ASSERT_NEAR(4.15171e+06, points[0].getX(), 5.0);
    ASSERT_NEAR(583682, points[0].getY(), 5.0);    
    ASSERT_NEAR(4.79047e+06, points[0].getZ(), 5.0);
}


TEST(RouteGeneratorTest, testReadRoutetfhrcFile)
{
    tf2_ros::Buffer tf_buffer;
    carma_wm::WorldModelConstPtr wm;
    route::RouteGeneratorWorker worker(tf_buffer);
    worker.set_route_file_path("../resource/route/");
    cav_srvs::GetAvailableRoutesRequest req;
    cav_srvs::GetAvailableRoutesResponse resp;
    ASSERT_TRUE(worker.get_available_route_cb(req, resp));
    std::cout << "Available Route : " << resp.availableRoutes.size() << "\n";
    ASSERT_EQ(4, resp.availableRoutes.size());
    for(auto i = 0; i < resp.availableRoutes.size();i++)    
    {
        if(resp.availableRoutes[i].route_id  == "tfhrc_test_route")
        {
            std::cout <<"C-HUB : " << resp.availableRoutes[i].route_name << "\n";
            auto points = worker.load_route_destinations_in_ecef("tfhrc_test_route");
            std::cout << "Point Size : " << points.size()<<"\n";
            ASSERT_EQ(5, points.size());
            ASSERT_NEAR(1106580, points[0].getX(), 5.0);
            ASSERT_NEAR(894697, points[0].getY(), 5.0);  
            ASSERT_NEAR(-6196590, points[0].getZ(), 5.0);
        }
   }
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
