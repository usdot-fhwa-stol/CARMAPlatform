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

#include "port_drayage_plugin/port_drayage_worker.h"
#include "port_drayage_plugin/port_drayage_state_machine.h"
#include "port_drayage_plugin/port_drayage_plugin.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <sstream>

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/WMTestLibForGuidance.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <novatel_gps_msgs/Inspva.h>

TEST(PortDrayageTest, testComposeArrivalMessage)
{
    // Test arrival message for pdw initialized with cargo
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    // Set host vehicle's current gps lat/lon position
    novatel_gps_msgs::Inspva gps_reading;
    gps_reading.latitude = 38.95622708;
    gps_reading.longitude = -77.15066142;
    novatel_gps_msgs::InspvaConstPtr gps_reading_pointer(new novatel_gps_msgs::Inspva(gps_reading));
    pdw.set_current_gps_position(gps_reading_pointer);

    cav_msgs::MobilityOperation msg = pdw.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg.header.sender_id);
    ASSERT_FALSE(msg.strategy_params.empty());

    std::istringstream strstream(msg.strategy_params);
    using boost::property_tree::ptree;
    ptree pt;
    boost::property_tree::json_parser::read_json(strstream, pt);

    unsigned long cmv_id = pt.get<unsigned long>("cmv_id");
    std::string cargo_id = pt.get<std::string>("cargo_id");
    std::string operation = pt.get<std::string>("operation");
    bool has_cargo = pt.get<bool>("cargo");
    double vehicle_longitude = pt.get<double>("location.longitude");
    double vehicle_latitude = pt.get<double>("location.latitude");

    ASSERT_EQ(123, cmv_id);
    ASSERT_EQ("321", cargo_id);
    ASSERT_EQ("ARRIVED_AT_DESTINATION", operation);
    ASSERT_TRUE(has_cargo);
    ASSERT_NEAR(389562270.8, vehicle_latitude, 0.001);
    ASSERT_NEAR(-771506614.2, vehicle_longitude, 0.001);

    // Test arrival message for pdw initialized without cargo
    port_drayage_plugin::PortDrayageWorker pdw2{
        123, // CMV ID 
        "", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    pdw2.set_current_gps_position(gps_reading_pointer);

    cav_msgs::MobilityOperation msg2 = pdw2.compose_arrival_message();

    ASSERT_EQ("carma/port_drayage", msg2.strategy);
    ASSERT_EQ("TEST_CARMA_HOST_ID", msg2.header.sender_id);
    ASSERT_FALSE(msg2.strategy_params.empty());

    std::istringstream strstream2(msg2.strategy_params);
    ptree pt2;
    boost::property_tree::json_parser::read_json(strstream2, pt2);

    unsigned long cmv_id2 = pt2.get<unsigned long>("cmv_id");
    std::string operation2 = pt2.get<std::string>("operation");
    bool has_cargo2 = pt2.get<bool>("cargo");
    double vehicle_longitude2 = pt2.get<double>("location.longitude");
    double vehicle_latitude2 = pt2.get<double>("location.latitude");

    ASSERT_EQ(123, cmv_id2);
    ASSERT_EQ(0, pt2.count("cargo_id")); // Broadcasted arrival message should not include cargo_id since vehicle is not carrying cargo
    ASSERT_EQ("ARRIVED_AT_DESTINATION", operation2);
    ASSERT_FALSE(has_cargo2); // False since vehicle is not currently carrying cargo
    ASSERT_NEAR(389562270.8, vehicle_latitude2, 0.001);
    ASSERT_NEAR(-771506614.2, vehicle_longitude2, 0.001);
}

TEST(PortDrayageTest, testCheckStop1)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Port Drayage Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_TRUE(stopped);
}

TEST(PortDrayageTest, testCheckStop2)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "321", // Cargo ID
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 2.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Port Drayage Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testCheckStop3)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 0.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Route Following Plugin";
    plan.maneuvers.push_back(mvr);
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testCheckStop4)
{
    ros::Time::init();
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    geometry_msgs::TwistStamped twist;
    twist.twist.linear.x = 2.0;
    geometry_msgs::TwistStampedConstPtr twistptr{ new geometry_msgs::TwistStamped{twist}};
    pdw.set_current_speed(twistptr);

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver mvr;
    mvr.type = cav_msgs::Maneuver::STOP_AND_WAIT;
    mvr.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "Route Following Plugin";
    plan.maneuvers.push_back(mvr);
    cav_msgs::ManeuverPlanConstPtr planptr{ new cav_msgs::ManeuverPlan{plan}};
    pdw.set_maneuver_plan(planptr);

    bool stopped = pdw.check_for_stop(planptr, twistptr);

    ASSERT_FALSE(stopped);
}

TEST(PortDrayageTest, testStateMachine)
{
    port_drayage_plugin::PortDrayageStateMachine pdsm;

    // Verify system initial state
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::INACTIVE, pdsm.get_state());

    // Startup CARMA system and begin automation
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::DRAYAGE_START);

    // Verify that we are en route to our next destination (port)
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdsm.get_state());

    // Notify state machine we've arrived
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::ARRIVED_AT_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::AWAITING_DIRECTION, pdsm.get_state());

    // Notify state machine we've recieved the next destination
    pdsm.process_event(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION);
    ASSERT_EQ(port_drayage_plugin::PortDrayageState::EN_ROUTE, pdsm.get_state());

    // Rest of the state machine to be implemented and validated in future stories
}

TEST(PortDrayageTest, testEstimateDistanceToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_distance_to_stop(10.0, 1.0), 50.0);
}

TEST(PortDrayageTest, testEstimateTimeToStop)
{
    ASSERT_EQ(port_drayage_plugin::estimate_time_to_stop(10.0, 1.0), 20.0);
}

TEST(PortDrayageTest, testPlanManeuverCb)
{
    cav_srvs::PlanManeuversRequest req;
    cav_srvs::PlanManeuversResponse resp;

    port_drayage_plugin::PortDrayagePlugin pdp{nullptr, nullptr};
    pdp.declaration = 1;

    ASSERT_EQ(pdp.plan_maneuver_cb(req,resp),false);

    geometry_msgs::Twist _cur_speed;
    _cur_speed.linear.x = 1;
    _cur_speed.linear.y = 2;

    pdp._cur_speed = _cur_speed;

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;    

    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.pose = pose;

    std::shared_ptr<geometry_msgs::PoseStamped> curr_pose_ = std::make_shared<geometry_msgs::PoseStamped>(pose_stamped);
    pdp.curr_pose_ = curr_pose_;

    std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

    auto pl1 = carma_wm::test::getPoint(0, 0, 0);
    auto pl2 = carma_wm::test::getPoint(0, 1, 0);
    auto pl3 = carma_wm::test::getPoint(0, 2, 0);
    auto pr1 = carma_wm::test::getPoint(1, 0, 0);
    auto pr2 = carma_wm::test::getPoint(1, 1, 0);
    auto pr3 = carma_wm::test::getPoint(1, 2, 0);

    std::vector<lanelet::Point3d> left_1 = { pl1, pl2 };
    std::vector<lanelet::Point3d> right_1 = { pr1, pr2 };
    auto ll_1 = carma_wm::test::getLanelet(left_1, right_1, lanelet::AttributeValueString::SolidSolid, lanelet::AttributeValueString::Dashed);

    std::vector<lanelet::Point3d> left_2 = { pl2, pl3 };
    std::vector<lanelet::Point3d> right_2 = { pr2, pr3 };
    auto ll_2 = carma_wm::test::getLanelet(left_2, right_2);

    // 1. Confirm all pointers are false (done above)
    // Ensure that none of the returned pointers are valid if the map has not been set
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_FALSE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());


    lanelet::Id stop_line_id = lanelet::utils::getId();
    lanelet::LineString3d virtual_stop_line(stop_line_id, {pl2, pr2});
    // Creat passing control line for solid dashed line
    std::shared_ptr<lanelet::StopRule> stop_and_wait(new lanelet::StopRule(lanelet::StopRule::buildData(
    lanelet::utils::getId(), { virtual_stop_line }, { lanelet::Participants::Vehicle })));

    ll_1.addRegulatoryElement(stop_and_wait);


    // 2. Build map but do not assign
    // Create basic map and verify that the map and routing graph can be build, but the route remains false
    lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2 }, {});
    
    // // traffic lights are created from a linestring that shows a traffic light and optionally a stop line.
    // lanelet::LineString3d stop_rule = lanelet::LineString3d(lanelet::utils::getId(), {lanelet::Point3d{lanelet::utils::getId(), 1, 0, 0}, lanelet::Point3d{lanelet::utils::getId(), 1, 1, 0}, lanelet::Point3d{lanelet::utils::getId(), 1, 2, 0}});
    // trafficLight.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::TrafficLight;
    // // this creates our traffic light. Regelems are passed around as shared pointers.
    // lanelet::RegulatoryElementPtr trafficLightRegelem = lanelet::TrafficLight::make(lanelet::utils::getId(), {}, {trafficLight});

    // // 3. Build routing graph but do not assign
    // // Build routing graph from map
    lanelet::traffic_rules::TrafficRulesUPtr traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::VehicleCar);

    lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

    // 4. Generate route
    auto optional_route = map_graph->getRoute(ll_1, ll_2);
    ASSERT_TRUE((bool)optional_route);

    lanelet::routing::Route route = std::move(*optional_route);
    carma_wm::LaneletRoutePtr route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
    // 5. Try to set route without map and ensure it passes
    cmw->setRoute(route_ptr);
    // 6. getRoute is true but other pointers are false
    ASSERT_FALSE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_FALSE((bool)cmw->getMapRoutingGraph());

    cmw->setMap(map);
    // 8. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());
    // 9. Call setRoute again to confirm no errors
    cmw->setRoute(route_ptr);
    // 10. All pointers exist
    ASSERT_TRUE((bool)cmw->getMap());
    ASSERT_TRUE((bool)cmw->getRoute());
    ASSERT_TRUE((bool)cmw->getMapRoutingGraph());

    pdp.wm_ = cmw;

    ASSERT_EQ(pdp.plan_maneuver_cb(req,resp),true);

    ros::Time time;
    time.sec = 1;
    time.nsec = 0;

    cav_msgs::Maneuver maneuver = pdp.compose_stop_and_wait_maneuver_message(1, 2, 1, 2, 1, time, 10);
    ASSERT_EQ(maneuver.stop_and_wait_maneuver.end_time,time + ros::Duration(15));

    maneuver = pdp.compose_lane_following_maneuver_message(1, 2, 1, 2, 1, time);
    ASSERT_NEAR(maneuver.lane_following_maneuver.end_time.sec, 1, 0.0001);


}

TEST(PortDrayageTest, testInboundMobilityOperation)
{
    // Create PortDrayageWorker object with _cmv_id of 123
    port_drayage_plugin::PortDrayageWorker pdw{
        123, // CMV ID 
        "321", // Cargo ID 
        "TEST_CARMA_HOST_ID", 
        std::function<void(cav_msgs::MobilityOperation)>(), 
        1.0};

    // Create a MobilityOperationConstPtr with a cmv_id that is intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    cav_msgs::MobilityOperation mobility_operation_msg;
    mobility_operation_msg.strategy = "carma/port_drayage";
    mobility_operation_msg.strategy_params = "{ \"cmv_id\": \"123\", \"cargo_id\": \"321\", \"cargo\": \"false\", \"location\"\
        : { \"latitude\": \"389554377\", \"longitude\": \"-771503421\" }, \"destination\": { \"latitude\"\
        : \"389550038\", \"longitude\": \"-771481983\" }, \"operation\": \"MOVING_TO_LOADING_AREA\", \"action_id\"\
        : \"32\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr(new cav_msgs::MobilityOperation(mobility_operation_msg));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr);
    
    ASSERT_EQ("321", *pdw._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("MOVING_TO_LOADING_AREA", pdw._latest_mobility_operation_msg.operation);
    ASSERT_EQ(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION, pdw._latest_mobility_operation_msg.port_drayage_event_type);
    ASSERT_EQ(false, pdw._latest_mobility_operation_msg.has_cargo);
    ASSERT_EQ(-77.1503421, *pdw._latest_mobility_operation_msg.start_longitude);
    ASSERT_EQ(38.9554377, *pdw._latest_mobility_operation_msg.start_latitude);
    ASSERT_EQ(-77.1481983, *pdw._latest_mobility_operation_msg.dest_longitude);
    ASSERT_EQ(38.9550038, *pdw._latest_mobility_operation_msg.dest_latitude);
    ASSERT_EQ("32", *pdw._latest_mobility_operation_msg.current_action_id);

    // Create a MobilityOperationConstPtr with a cmv_id that is not intended for this specific vehicle
    // Note: The strategy_params using the schema for messages of this type that have strategy "carma/port_drayage"
    cav_msgs::MobilityOperation mobility_operation_msg2;
    mobility_operation_msg2.strategy = "carma/port_drayage";
    mobility_operation_msg2.strategy_params = "{ \"cmv_id\": \"444\", \"cargo_id\": \"567\", \"cargo\": \"true\", \"location\"\
        : { \"latitude\": \"489554377\", \"longitude\": \"-671503421\" }, \"destination\": { \"latitude\"\
        : \"489550038\", \"longitude\": \"-571481983\" }, \"operation\": \"MOVING_FROM_LOADING_AREA\", \"action_id\"\
        : \"44\" }";
    cav_msgs::MobilityOperationConstPtr mobility_operation_msg_ptr2(new cav_msgs::MobilityOperation(mobility_operation_msg2));
    pdw.on_inbound_mobility_operation(mobility_operation_msg_ptr2);

    ASSERT_EQ("321", *pdw._latest_mobility_operation_msg.cargo_id);
    ASSERT_EQ("MOVING_TO_LOADING_AREA", pdw._latest_mobility_operation_msg.operation);
    ASSERT_EQ(port_drayage_plugin::PortDrayageEvent::RECEIVED_NEW_DESTINATION, pdw._latest_mobility_operation_msg.port_drayage_event_type);
    ASSERT_EQ(false, pdw._latest_mobility_operation_msg.has_cargo);
    ASSERT_EQ(-77.1503421, *pdw._latest_mobility_operation_msg.start_longitude);
    ASSERT_EQ(38.9554377, *pdw._latest_mobility_operation_msg.start_latitude);
    ASSERT_EQ(-77.1481983, *pdw._latest_mobility_operation_msg.dest_longitude);
    ASSERT_EQ(38.9550038, *pdw._latest_mobility_operation_msg.dest_latitude);
    ASSERT_EQ("32", *pdw._latest_mobility_operation_msg.current_action_id);
}

// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
