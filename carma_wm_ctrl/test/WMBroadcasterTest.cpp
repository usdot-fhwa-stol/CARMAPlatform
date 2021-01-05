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

#include <gmock/gmock.h>
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm/TrafficControl.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <carma_utils/testing/TestHelpers.h>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>
#include <algorithm>

#include <cav_msgs/Route.h>
#include <cav_msgs/TrafficControlMessage.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>
#include <geometry_msgs/PoseStamped.h>

#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using carma_utils::timers::testing::TestTimer;
using carma_utils::timers::testing::TestTimerFactory;

namespace carma_wm_ctrl

{

TEST(WMBroadcaster, Constructor)
{
  WMBroadcaster([](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const autoware_lanelet2_msgs::MapBin& map_bin) {},
   [](const cav_msgs::TrafficControlRequest& control_msg_pub_){}, [](const cav_msgs::CheckActiveGeofence& active_pub_){}, std::make_unique<TestTimerFactory>());  // Create broadcaster with test timers. Having this check helps
                                                        // verify that the timers do not crash on destruction
}
TEST(WMBroadcaster, baseMapCallback)
{
  ros::Time::setNow(ros::Time(0));  // Set current time


  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);

  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);

  ASSERT_EQ(1, base_map_call_count);
}

// here test the proj string transform test
TEST(WMBroadcaster, getAffectedLaneletOrAreasFromTransform)
{
  using namespace lanelet::units::literals;
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Get and convert map to binary message
  /////
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(base_map_proj);

  // create the geofence request
  cav_msgs::TrafficControlMessageV01 gf_msg;
  gf_msg.geometry.proj = geofence_proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = -8.5; pt.y = -9.5; pt.z = 0; // straight geofence line across 2 lanelets
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = -8.5; pt.y = -8.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  
  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2);
  ASSERT_EQ(affected_parts[0].id(), 10002);
  ASSERT_EQ(affected_parts[1].id(), 10001);
  // check points that are outside, on the edge, and on the point that makes up the lanelets
  pt.x = -20; pt.y = -10; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = -9; pt.y = -8.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0; pt.y = 0; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2); // newly added ones should not be considered to be on the lanelet
}

// here test assuming the georeference proj strings are the same
TEST(WMBroadcaster, getAffectedLaneletOrAreasOnlyLogic)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        
        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Get and convert map to binary message
  /////
  auto map = carma_wm::getBroadcasterTestMap();

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  cav_msgs::TrafficControlMessageV01 gf_msg;
  // Check if error are correctly being thrown
  EXPECT_THROW(wmb.getAffectedLaneletOrAreas(gf_msg), lanelet::InvalidObjectStateError);
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);
  
  EXPECT_THROW(wmb.getAffectedLaneletOrAreas(gf_msg), lanelet::InvalidObjectStateError);
  
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // create the control message's relevant parts
  gf_msg.geometry.proj = proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.75; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  lanelet::ConstLaneletOrAreas affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 0); // this is 0 because there will never be geofence with only 1 pt
                                       // if there is, it won't apply to the map as it doesn't have any direction information, 
                                       // which makes it confusing for overlapping lanelets
  pt.x = 1.75; pt.y = 0.45; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 0); // although there are two points in the same lanelet,
                                       // lanelet and the two points are not in the same direction

  gf_msg.geometry.nodes.pop_back();
  pt.x = 1.75; pt.y = 0.55; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 1); // because two points are in one geofence, it will be recorded now
  gf_msg.geometry.nodes.pop_back();
  gf_msg.geometry.nodes.pop_back();

  pt.x = 0.5; pt.y = 0.5; pt.z = 0;    // first of series geofence points across multiple lanelets
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.1; pt.z = 0;    // adding point in the next lanelet
  gf_msg.geometry.nodes.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg); 
  ASSERT_EQ(affected_parts.size(), 3);    // although (0.5,1.1) is in another overlapping lanelet (llt_unreg)
                                          // that lanelet is disjoint/doesnt have same direction/not successor of the any lanelet
  
  pt.x = 1.5; pt.y = 2.1; pt.z = 0;    // adding further points in different lanelet narrowing down our direction
  gf_msg.geometry.nodes.push_back(pt);
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 3);    // now they are actually 3 different lanelets because we changed direction
  std::vector<lanelet::Id> affected_parts_ids;
  for (auto i = 0; i < affected_parts.size(); i ++)
  {
    affected_parts_ids.push_back(affected_parts[i].id());
  }
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10003) != affected_parts_ids.end()); //due to race condition
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10000) != affected_parts_ids.end());
  ASSERT_TRUE(std::find(affected_parts_ids.begin(), affected_parts_ids.end(), 10006) != affected_parts_ids.end());

  // check points that are outside, on the edge, and on the point that makes up the lanelets
  pt.x = 0.5; pt.y = 0; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 1.0; pt.y = 0; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 10; pt.y = 10; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  
  affected_parts = wmb.getAffectedLaneletOrAreas(gf_msg);
  ASSERT_EQ(affected_parts.size(), 2); // they should not be considered to be on the lanelet
}

TEST(WMBroadcaster, geofenceCallback)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>(Geofence());

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Starts at 2
                                 ros::Duration(1.1),  // Ends at by 3.1
                                 ros::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2)));
  // convert to ros msg
  cav_msgs::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  cav_msgs::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;

  ros::Time::setNow(ros::Time(0));  // Set current time

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);



  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(4, map->laneletLayer.size());  // Verify the map can be decoded
        base_map_call_count++;
      },
      [&](const autoware_lanelet2_msgs::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        ASSERT_EQ(data_received->id_, curr_id);
        ASSERT_EQ(data_received->remove_list_.size(), 0);
        ASSERT_EQ(data_received->update_list_.size(), 0);
        active_call_count.store(active_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

 // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46645851394806215 +lon_0=-76.16907903057393980 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(base_map_proj);

  cav_msgs::TrafficControlMessage gf_msg;
  gf_msg.choice = cav_msgs::TrafficControlMessage::RESERVED;

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());

  wmb.geofenceCallback(gf_msg); 

  ros::Time::setNow(ros::Time(1.0));  // Set current time

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());

  // Verify that nothing is happening when reserved
  wmb.geofenceCallback(gf_msg); 
  ros::Time::setNow(ros::Time(2.1));// Geofence should have started by now

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  
  gf_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;
  gf_msg.tcmV01 = msg_v01;

  // every control message needs associated control request id
  cav_msgs::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  std::shared_ptr<j2735_msgs::Id64b> req_id = std::make_shared<j2735_msgs::Id64b>(j2735_msgs::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  gf_msg.tcmV01.reqid = *req_id;

  // check geofence with no applicable points
  ros::Time::setNow(ros::Time(0));
  wmb.geofenceCallback(gf_msg);
  ros::Time::setNow(ros::Time(2.1));  // Set current time
  ASSERT_FALSE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(0, active_call_count.load());

  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = -8.5; pt.y = -9.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = -8.5; pt.y = -8.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  msg_v01.reqid = *req_id;
  gf_msg.tcmV01 = msg_v01;

  ros::Time::setNow(ros::Time(0));
  // check how many times map_update is called so far
  // calling again with same id should not have an effect
  wmb.geofenceCallback(gf_msg);
  ros::Time::setNow(ros::Time(2.1));  // Set current time
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());

  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(),  curr_id.end(), msg_v01.id.id.begin());
  gf_msg.tcmV01 = msg_v01;
  wmb.geofenceCallback(gf_msg);

  // check if different geofence id is working
  ros::Time::setNow(ros::Time(3.0)); // right before finishing at 3.1
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(2, active_call_count.load());

}
  
TEST(WMBroadcaster, routeCallbackMessage) 
{
  cav_msgs::Route route_msg;
  
  route_msg.route_path_lanelet_ids.push_back(1346);
  route_msg.route_path_lanelet_ids.push_back(1349);

  ROS_INFO_STREAM("This is a test: ");
  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback  
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);

        ASSERT_EQ(2, map->laneletLayer.size());  // Verify the map can be decoded

        base_map_call_count++;
      }, [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());
 
  
  //Test throw exceptions
  ASSERT_THROW(wmb.routeCallbackMessage(route_msg), lanelet::InvalidObjectStateError);
  ROS_INFO_STREAM("Throw Exceptions Test Passed.");
  
  // Load vector map from a file start 
  std::string file = "resource/test_vector_map1.osm";
  int projector_type = 0;
  std::string target_frame;
  lanelet::ErrorMessages load_errors;
  // Parse geo reference info from the original lanelet map (.osm)
  lanelet::io_handlers::AutowareOsmParser::parseMapParams(file, &projector_type, &target_frame);

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());

  lanelet::LaneletMapPtr map = lanelet::load(file, local_projector, &load_errors);

  if (map->laneletLayer.size() == 0)
  {
    FAIL() << "Input map does not contain any lanelets";
  }
  // apply loaded map to WMBroadcaster
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);
  ASSERT_EQ(1, base_map_call_count);

  //Test target_frame value
  std_msgs::String target;
  target.data = target_frame;
  wmb.geoReferenceCallback(target);
  ASSERT_FALSE(target_frame.empty());
  // loading end
  
  cav_msgs::TrafficControlRequest coRe;

  ///// Test without user defined route callback
  coRe = wmb.controlRequestFromRoute(route_msg);

  ASSERT_TRUE(coRe.tcrV01.bounds.size() > 0);

}

TEST(WMBroadcaster, addAndRemoveGeofence)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  size_t map_update_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      }, 
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map update callback
        map_update_call_count++;
      }, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  //////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  
  cav_msgs::TrafficControlMessageV01 gf_msg;
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->regulatory_element_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.geometry.proj = proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_msg);

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 2);
  ASSERT_EQ(gf_ptr->affected_parts_[1].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 1);
  /*Analyze prev_regems_.size()*/
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 2);
  ASSERT_EQ(gf_ptr->prev_regems_[0].first, 10007);
  ASSERT_EQ(gf_ptr->prev_regems_[1].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 2);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0);

  // we can check if the removeGeofence worked, by using addGeofence again and if the original is there again
  wmb.addGeofence(gf_ptr);
  ASSERT_EQ(map_update_call_count, 3);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 2);
  ASSERT_EQ(gf_ptr->prev_regems_[1].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[1].second->id(), old_speed_limit->id());

}

TEST(WMBroadcaster, GeofenceBinMsgTest)
{
  using namespace lanelet::units::literals;
  // Set the environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());
  
  /////
  // Set up the map (add relevant regulatory elements)
  /////
  auto map = carma_wm::getBroadcasterTestMap();
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::DigitalSpeedLimitPtr old_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_speed_limit->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_speed_limit); // added a speed limit to first llt
  
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 1);
  ASSERT_TRUE(map->regulatoryElementLayer.exists(old_speed_limit->id()));
  ASSERT_EQ(map->regulatoryElementLayer.size(), 1);
  ASSERT_EQ(map->laneletLayer.findUsages(old_speed_limit).size(), 1);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_speed_limit->id());//should be 10045 old speed limit's id
  
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // Create the geofence object
  auto gf_ptr = std::make_shared<carma_wm_ctrl::Geofence>(carma_wm_ctrl::Geofence());
  gf_ptr->id_ = boost::uuids::random_generator()();
  
  cav_msgs::TrafficControlMessageV01 gf_msg;
  lanelet::DigitalSpeedLimitPtr new_speed_limit = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(map->regulatoryElementLayer.uniqueId(), 10_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  gf_ptr->regulatory_element_ = new_speed_limit;
  // create the control message's relevant parts to fill the object
  gf_msg.geometry.proj = proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  gf_msg.geometry.nodes.push_back(pt);
  gf_ptr->affected_parts_ = wmb.getAffectedLaneletOrAreas(gf_msg);

  ASSERT_EQ(gf_ptr->affected_parts_.size(), 2);
  ASSERT_EQ(gf_ptr->affected_parts_[1].id(), 10000);
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements()[0]->id(), old_speed_limit->id()); // old speed limit
  ASSERT_EQ(gf_ptr->affected_parts_[1].regulatoryElements().size(), 4); // old speed limit and other map conforming regulations
  // process the geofence and change the map

  // flow for adding geofence to the map
  wmb.addGeofence(gf_ptr);
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_obj_msg;

  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data, &gf_obj_msg);
  // at map users
  auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_obj_msg, data_received);
  ASSERT_EQ(data_received->id_, gf_ptr->id_);
  ASSERT_EQ(gf_ptr->remove_list_.size(), 2);
  ASSERT_EQ(data_received->remove_list_.size(), 2); // old_speed_limit
  ASSERT_TRUE(data_received->remove_list_[0].second->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) ==0 );
  ASSERT_EQ(data_received->update_list_.size(), 2); // geofence tags 2 lanelets
  ASSERT_EQ(data_received->update_list_[1].first, 10000);

  // we can see that the gf_ptr->now would have the prev speed limit of 5_mph that affected llt 10000
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 2);
  ASSERT_EQ(gf_ptr->prev_regems_[1].first, 10000);
  ASSERT_EQ(gf_ptr->prev_regems_[1].second->id(), old_speed_limit->id());

  // now suppose the geofence is finished being used, we have to revert the changes
  wmb.removeGeofence(gf_ptr);
  ASSERT_EQ(gf_ptr->prev_regems_.size(), 0); // should be reset
  // from broadcaster
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  auto send_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data_revert, &gf_msg_revert);
  // at map users
  auto rec_data_revert = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(gf_msg_revert, rec_data_revert);

  // previously added update_list_ should be tagged for removal, vice versa
  ASSERT_EQ(rec_data_revert->remove_list_.size(), 2);
  ASSERT_EQ(rec_data_revert->remove_list_.size(), data_received->update_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), data_received->remove_list_.size());
  ASSERT_EQ(rec_data_revert->update_list_.size(), 2);
  ASSERT_EQ(rec_data_revert->update_list_[1].first, 10000);
  ASSERT_EQ(rec_data_revert->update_list_[1].second->id(), old_speed_limit->id());
  
}

TEST(WMBroadcaster, RegulatoryPCLTest)
{
  // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf_ptr = std::make_shared<Geofence>(Geofence());

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf_ptr->id_ = curr_id;

  gf_ptr->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Starts at 2
                                 ros::Duration(1.1),  // Ends at by 3.1
                                 ros::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2)));
  // convert to ros msg
  cav_msgs::TrafficControlMessageV01 msg_v01;
  std::copy(gf_ptr->id_.begin(),  gf_ptr->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf_ptr->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf_ptr->schedules[0].schedule_end_;
  cav_msgs::DailySchedule daily_schedule;
  daily_schedule.begin = gf_ptr->schedules[0].control_start_;
  daily_schedule.duration = gf_ptr->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf_ptr->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf_ptr->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf_ptr->schedules[0].control_period_;

  ros::Time::setNow(ros::Time(0));  // Set current time

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  bool testing_forward_direction = false;
  bool testing_reverse_direction = false;

  // Get and convert map to binary message
  auto map = carma_wm::getBroadcasterTestMap();
  
  ASSERT_EQ(map->regulatoryElementLayer.size(), 0);
  // add regems
  lanelet::PassingControlLinePtr old_pcl = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(10082, {map->laneletLayer.get(10000).leftBound()}, {},
                                                     { lanelet::Participants::VehicleCar }));
  ASSERT_TRUE(old_pcl->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) == 0);
  ASSERT_EQ(map->laneletLayer.get(10000).regulatoryElements().size(), 0);
  map->update(map->laneletLayer.get(10000), old_pcl); // added a passing control line
  map->update(map->laneletLayer.get(10007), old_pcl);
  ASSERT_EQ(map->laneletLayer.find(10000)->regulatoryElements().front()->id(), old_pcl->id());
  ASSERT_EQ(map->laneletLayer.find(10007)->regulatoryElements().front()->id(), old_pcl->id());
  ASSERT_FALSE(old_pcl->passableFromLeft(lanelet::Participants::VehicleCar));
  ASSERT_TRUE(old_pcl->passableFromRight(lanelet::Participants::VehicleCar));

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [&](const autoware_lanelet2_msgs::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);

        ASSERT_EQ(data_received->id_, curr_id);

        // gather the new control lines from update list to test
        std::vector<lanelet::PassingControlLinePtr> control_lines;
        for (auto pair : data_received->update_list_)
        {
          auto factory_pcl = lanelet::RegulatoryElementFactory::create(pair.second->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(pair.second->constData()));
          lanelet::PassingControlLinePtr control_line = std::dynamic_pointer_cast<lanelet::PassingControlLine>(factory_pcl);
          control_lines.push_back(control_line);
        }
        
        if (testing_forward_direction)
        {
          ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines,
                                                 true, lanelet::Participants::VehicleCar));
          ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10000).leftBound(), control_lines,
                                                      false, lanelet::Participants::VehicleCar));
        }
        if (testing_reverse_direction)
        {
          ASSERT_FALSE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10005).leftBound(), control_lines, true,
                                                      lanelet::Participants::VehicleCar));
          ASSERT_TRUE(lanelet::PassingControlLine::boundPassable(map->laneletLayer.get(10005).leftBound(), control_lines, false,
                                                      lanelet::Participants::VehicleCar));
        }
       
        active_call_count.store(active_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

 autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // set the accessibility
  msg_v01.params_exists = true;
  j2735_msgs::TrafficControlVehClass veh_class;
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::PASSENGER_CAR;
  msg_v01.params.vclasses.push_back(veh_class);
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE;
  msg_v01.params.detail.lataffinity = cav_msgs::TrafficControlDetail::LEFT; // applies to the left boundaries of the 
  msg_v01.params.detail.latperm[0] = cav_msgs::TrafficControlDetail::NONE; // not accessible from left
  msg_v01.params.detail.latperm[1] = cav_msgs::TrafficControlDetail::PERMITTED; // accessible from right

  // create the control message's relevant parts to fill the object
  msg_v01.geometry.proj = proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets, thauto gf_ptr = std::make_shared<Geofence>(Geofence());ese correspond to id 10000, 10007
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;  
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);

  // register the geofence
  cav_msgs::TrafficControlMessage gf_msg;
  gf_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;

  // every control message needs associated control request id
  cav_msgs::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  std::shared_ptr<j2735_msgs::Id64b> req_id = std::make_shared<j2735_msgs::Id64b>(j2735_msgs::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  gf_msg.tcmV01 = msg_v01;
  testing_forward_direction = true;

  wmb.geofenceCallback(gf_msg);

  ros::Time::setNow(ros::Time(2.1));  // Set current time
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());

  testing_forward_direction = false;
  testing_reverse_direction = true;

  ros::Time::setNow(ros::Time(0));  // Reset time
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());
  msg_v01.geometry.nodes = {};
  pt.x = 0.5; pt.y = 0.75; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  gf_msg.tcmV01 = msg_v01;

  wmb.geofenceCallback(gf_msg);
  ros::Time::setNow(ros::Time(2.1));  // Set current time
  
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(2, active_call_count.load());
}

TEST(WMBroadcaster, geofenceFromMsgTest)
{
  using namespace lanelet::units::literals;
  // Start creating ROS msg
  cav_msgs::TrafficControlMessageV01 msg_v01;
  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::copy(curr_id.begin(),  curr_id.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = ros::Time(1);  // Schedule between 1 ...
  msg_v01.params.schedule.end = ros::Time(8);    // and 8
  cav_msgs::DailySchedule daily_schedule;
  daily_schedule.begin = ros::Duration(2);       // Starts at 2
  daily_schedule.duration = ros::Duration(1.1);  // Ends at by 3.1
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  ros::Duration(0);  // 0 offset for repetition start, so still starts at 2
  msg_v01.params.schedule.repeat.span = ros::Duration(1);     // Duration of 1 and interval of two so active durations are (2-3)
  msg_v01.params.schedule.repeat.period =  ros::Duration(2);

  // Get map and convert map to binary message
  auto map = carma_wm::getBroadcasterTestMap();

  // Set a basic environment  
  size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
        base_map_call_count++;
      },
      [](const autoware_lanelet2_msgs::MapBin& map_bin) {}, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));
  // Set the map
  wmb.baseMapCallback(map_msg_ptr);
  // Setting georeference otherwise, geofenceCallback will throw exception
  std_msgs::String sample_proj_string;
  std::string proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  sample_proj_string.data = proj_string;
  wmb.geoReferenceCallback(sample_proj_string);

  // create rest of control message's relevant parts to fill the object
  msg_v01.geometry.proj = proj_string;
  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets, these correspond to id 10000, 10007
  pt.x = 0.5; pt.y = 0.5; pt.z = 0;  
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 0.5; pt.y = 1.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  msg_v01.params_exists = true;
  
  // test maxspeed - config limit inactive
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 99;

  lanelet::Velocity limit = 80_mph;

  auto gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr max_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ROS_ERROR_STREAM("Speed Limit: "<<max_speed->speed_limit_.value());
  ASSERT_NEAR(max_speed->speed_limit_.value(), limit.value(),0.0001) ;//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_GE(max_speed->speed_limit_, 0_mph);//Check that the maximum speed limit is not smaller than 0_mph
  ROS_WARN_STREAM("Maximum speed limit is valid (1).");

  msg_v01.params.detail.maxspeed = -4;

  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  max_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_GE(max_speed->speed_limit_, 0_mph);//Check that the maximum speed limit is not smaller than 0_mph
  ROS_WARN_STREAM("Maximum speed limit is valid(2).");


  // test minspeed - config limit inactive
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MINSPEED_CHOICE;
  msg_v01.params.detail.minspeed = -4.0;


  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr min_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_GE(min_speed->speed_limit_, 0_mph);
  ROS_WARN_STREAM("Minimum speed limit is valid.(1)");


   msg_v01.params.detail.minspeed = 99.0;


  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  min_speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr->regulatory_element_);
  ASSERT_NEAR(min_speed->speed_limit_.value(), limit.value(), 0.0001) ;//Check that the minimum speed limit is not larger than 80_mph
  ASSERT_GE(min_speed->speed_limit_, 0_mph);
  ROS_WARN_STREAM("Minimum speed limit is valid.(2)");

  

// test maxspeed - config limit active
  wmb.setConfigSpeedLimit(55.0);//Set the config speed limit
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 0;
  auto gf_ptr2 = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr2->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr max_speed_cL = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr2->regulatory_element_);
  //ASSERT_NEAR(max_speed->speed_limit_.value(), 22.352, 0.00001);
  ASSERT_LE(max_speed_cL->speed_limit_, 80_mph);//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_EQ(max_speed_cL->speed_limit_, 55_mph);//Check that the maximum speed limit is equal to the configured limit
  ROS_WARN_STREAM("Maximum speed limit (config_limit enabled) is valid.");

   // test minspeed - config limit active
  wmb.setConfigSpeedLimit(55.0);//Set the config speed limit
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MINSPEED_CHOICE;
  msg_v01.params.detail.minspeed = 0;
  gf_ptr2 = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr2->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0);
  lanelet::DigitalSpeedLimitPtr min_speed_cL = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(gf_ptr2->regulatory_element_);
 // ASSERT_NEAR(min_speed->speed_limit_.value(), 22.352,  0.00001);
  ASSERT_LE(min_speed_cL->speed_limit_, 80_mph);//Check that the maximum speed limit is not larger than 80_mph
  ASSERT_GE(min_speed_cL->speed_limit_, 0_mph);
  ASSERT_EQ(min_speed_cL->speed_limit_, 55_mph);
  ROS_WARN_STREAM("Minimum speed limit (config_limit enabled) is valid.");




  // TEST passing control line
  // Test lataffinity
  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE;
  msg_v01.params.detail.lataffinity = cav_msgs::TrafficControlDetail::LEFT; // applies to the left boundaries of the 
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) == 0);
  ASSERT_TRUE(gf_ptr->pcl_affects_left_);

  msg_v01.params.detail.lataffinity = cav_msgs::TrafficControlDetail::RIGHT; // applies to the right boundaries of the 
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  ASSERT_TRUE(gf_ptr->pcl_affects_right_);

  msg_v01.params.detail.latperm[0] = cav_msgs::TrafficControlDetail::NONE; // not accessible from left
  msg_v01.params.detail.latperm[1] = cav_msgs::TrafficControlDetail::PERMITTED; // accessible from right

  // Test Participants
  j2735_msgs::TrafficControlVehClass veh_class;
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::ANY;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  lanelet::PassingControlLinePtr pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_EQ(pcl->right_participants_.size(), 3);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::PEDESTRIAN;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::Pedestrian) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::BICYCLE;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::Bicycle) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::MOTORCYCLE;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleMotorcycle) == 0);
  
  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::BUS;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleBus) == 0);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::LIGHT_TRUCK_VAN;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleCar) == 0);

  msg_v01.params.vclasses = {};
  veh_class.vehicle_class = j2735_msgs::TrafficControlVehClass::THREE_AXLE_SINGLE_UNIT_TRUCK;
  msg_v01.params.vclasses.push_back(veh_class);
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleTruck) == 0);
  ASSERT_EQ(pcl->left_participants_.size(), 0);
  
  msg_v01.params.detail.latperm[0] = cav_msgs::TrafficControlDetail::PERMITTED; // accessible from left
  msg_v01.params.detail.latperm[1] = cav_msgs::TrafficControlDetail::NONE; // not accessible from right
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->left_participants_.begin()->data(), lanelet::Participants::VehicleTruck) == 0);
  ASSERT_EQ(pcl->right_participants_.size(), 0);

  msg_v01.params.detail.latperm[0] = cav_msgs::TrafficControlDetail::EMERGENCYONLY; 
  msg_v01.params.detail.latperm[1] = cav_msgs::TrafficControlDetail::EMERGENCYONLY; 
  gf_ptr = wmb.geofenceFromMsg(msg_v01);
  pcl = std::dynamic_pointer_cast<lanelet::PassingControlLine>(gf_ptr->regulatory_element_);
  ASSERT_TRUE(strcmp(pcl->right_participants_.begin()->data(), lanelet::Participants::VehicleEmergency) == 0);
  ASSERT_TRUE(strcmp(pcl->left_participants_.begin()->data(), lanelet::Participants::VehicleEmergency) == 0);
  
}

TEST(WMBroadcaster, distToNearestActiveGeofence)
{
   // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>(Geofence());

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Starts at 2
                                 ros::Duration(1.1),  // Ends at by 3.1
                                 ros::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2)));
  // convert to ros msg
  cav_msgs::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  cav_msgs::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;

  ros::Time::setNow(ros::Time(0));  // Set current time

  // variables needed to test
  size_t base_map_call_count = 0;
  std::atomic<uint32_t> map_update_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  bool activated = false;

  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
      },
      [&](const autoware_lanelet2_msgs::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        map_update_call_count.store(map_update_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        if (activated)
          last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
        if (!activated)
          last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(base_map_proj);
  
  cav_msgs::TrafficControlMessage gf_msg;
  gf_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;

  // every control message needs associated control request id
  cav_msgs::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  route_msg.route_path_lanelet_ids.push_back(10001);
  route_msg.route_path_lanelet_ids.push_back(10002);
  route_msg.route_path_lanelet_ids.push_back(10003);
  std::shared_ptr<j2735_msgs::Id64b> req_id = std::make_shared<j2735_msgs::Id64b>(j2735_msgs::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.5; pt.y = 1.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 1.5; pt.y = 2.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());

  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 50;
  gf_msg.tcmV01 = msg_v01;

  // Make sure the geofence is active now
  ros::Time::setNow(ros::Time(0));
  activated = true;
  wmb.geofenceCallback(gf_msg);
  ros::Time::setNow(ros::Time(2.1));  // Set current time so that geofence is active
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, map_update_call_count.load());

  lanelet::BasicPoint2d curr_pos = {1.5,0.5};
  double nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0.5, 0.0001);

  curr_pos = {0.5,0.5};
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 1.5, 0.0001);

  curr_pos = {1.5,1.5};  // it is currently on an active geofence
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0.5, 0.0001);  // it should point the next

  curr_pos = {1.5,3.5};  // it is currently not on any lanelet
  EXPECT_THROW(wmb.distToNearestActiveGeofence(curr_pos), std::invalid_argument);

  activated = false;
  ros::Time::setNow(ros::Time(3.2));  // Geofences deactivate now
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_inactive_gf));
  ASSERT_EQ(2, map_update_call_count.load());

  curr_pos = {1.5,1.5};  // it is currently on an active geofence
  nearest_gf_dist = wmb.distToNearestActiveGeofence(curr_pos);
  ASSERT_NEAR(nearest_gf_dist, 0, 0.0001);  // it should point the next
}


TEST(WMBroadcaster, currentLocationCallback)
{

   // Test adding then evaluate if the calls to active and inactive are done correctly
  auto gf = std::make_shared<Geofence>(Geofence());

  boost::uuids::uuid curr_id = boost::uuids::random_generator()(); 
  std::size_t curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  gf->id_ = curr_id;

  gf->schedules.push_back(carma_wm_ctrl::GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                                 ros::Time(8),
                                 ros::Duration(2),    // Starts at 2
                                 ros::Duration(1.1),  // Ends at by 3.1
                                 ros::Duration(0),    // 0 offset for repetition start, so still starts at 2
                                 ros::Duration(1),    // Duration of 1 and interval of two so active durations are (2-3)
                                 ros::Duration(2)));
  // convert to ros msg
  cav_msgs::TrafficControlMessageV01 msg_v01;
  std::copy(gf->id_.begin(),  gf->id_.end(), msg_v01.id.id.begin());
  msg_v01.params.schedule.start = gf->schedules[0].schedule_start_;
  msg_v01.params.schedule.end = gf->schedules[0].schedule_end_;
  cav_msgs::DailySchedule daily_schedule;
  daily_schedule.begin = gf->schedules[0].control_start_;
  daily_schedule.duration = gf->schedules[0].control_duration_;
  msg_v01.params.schedule.between.push_back(daily_schedule);
  msg_v01.params.schedule.repeat.offset =  gf->schedules[0].control_offset_;
  msg_v01.params.schedule.repeat.span =  gf->schedules[0].control_span_;
  msg_v01.params.schedule.repeat.period =  gf->schedules[0].control_period_;

  ros::Time::setNow(ros::Time(0));  // Set current time



  

 // variables needed to test
  std::atomic<uint32_t> map_update_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  bool activated = false;

//Create input message
  geometry_msgs::PoseStamped input_msg;

  //Input message coordinates
  input_msg.pose.position.x = 1.5;
  input_msg.pose.position.y = 1.5;
  input_msg.pose.position.z = 0.0;



 size_t base_map_call_count = 0;
  WMBroadcaster wmb(
      [&](const autoware_lanelet2_msgs::MapBin& map_bin) {
        // Publish map callback
        lanelet::LaneletMapPtr map(new lanelet::LaneletMap);
        lanelet::utils::conversion::fromBinMsg(map_bin, map);
      },
      [&](const autoware_lanelet2_msgs::MapBin& geofence_bin) {
        auto data_received = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
        carma_wm::fromBinMsg(geofence_bin, data_received);
        map_update_call_count.store(map_update_call_count.load() + 1);
        // atomic is not working for boost::uuids::uuid, so hash it
        if (activated)
          last_active_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
        if (!activated)
          last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(data_received->id_));
      }, [](const cav_msgs::TrafficControlRequest& control_msg_pub_){},
      [](const cav_msgs::CheckActiveGeofence& active_pub_){},
      std::make_unique<TestTimerFactory>());

   //Test throw exceptions
  ASSERT_THROW(wmb.currentLocationCallback(input_msg), lanelet::InvalidObjectStateError);
  ROS_INFO_STREAM("Throw Exceptions Test Passed.");

  // Get and convert map to binary message
  auto map = carma_wm::getDisjointRouteMap();
  autoware_lanelet2_msgs::MapBin msg;
  lanelet::utils::conversion::toBinMsg(map, &msg);
  autoware_lanelet2_msgs::MapBinConstPtr map_msg_ptr(new autoware_lanelet2_msgs::MapBin(msg));

  // Trigger basemap callback
  wmb.baseMapCallback(map_msg_ptr);

  // Setting georeferences
  // geofence's origin (0,0) is at base_map's (10,10)
  std::string base_map_proj_string, geofence_proj_string;
  std_msgs::String base_map_proj;
  base_map_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  geofence_proj_string = "+proj=tmerc +lat_0=39.46636844371259 +lon_0=-76.16919523566943 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs";
  base_map_proj.data = base_map_proj_string;
  wmb.geoReferenceCallback(base_map_proj);
  
  cav_msgs::TrafficControlMessage gf_msg;
  gf_msg.choice = cav_msgs::TrafficControlMessage::TCMV01;
  // create the geofence request
  msg_v01.geometry.proj = geofence_proj_string;

  // every control message needs associated control request id
  cav_msgs::Route route_msg;
  route_msg.route_path_lanelet_ids.push_back(10000);
  route_msg.route_path_lanelet_ids.push_back(10001);
  route_msg.route_path_lanelet_ids.push_back(10002);
  route_msg.route_path_lanelet_ids.push_back(10003);
  std::shared_ptr<j2735_msgs::Id64b> req_id = std::make_shared<j2735_msgs::Id64b>(j2735_msgs::Id64b());
  wmb.controlRequestFromRoute(route_msg, req_id);
  msg_v01.reqid = *req_id;

  // set the points
  cav_msgs::PathNode pt;
  // check points that are inside lanelets
  pt.x = 1.5; pt.y = 1.5; pt.z = 0; // straight geofence line across 2 lanelets
  msg_v01.geometry.nodes.push_back(pt);
  pt.x = 1.5; pt.y = 2.5; pt.z = 0;
  msg_v01.geometry.nodes.push_back(pt);
  // update id to continue testing
  curr_id = boost::uuids::random_generator()(); 
  curr_id_hashed = boost::hash<boost::uuids::uuid>()(curr_id);
  std::copy(curr_id.begin(), curr_id.end(), msg_v01.id.id.begin());

  msg_v01.params.detail.choice = cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE;
  msg_v01.params.detail.maxspeed = 50;
  gf_msg.tcmV01 = msg_v01;

  // Make sure the geofence is active now
  ros::Time::setNow(ros::Time(0));
  activated = true;
  wmb.geofenceCallback(gf_msg);
  ros::Time::setNow(ros::Time(2.1));  // Set current time so that geofence is active
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_active_gf));
  ASSERT_EQ(1, map_update_call_count.load());

 // wmb.addGeofence(gf);

  std::unordered_set<lanelet::Id> active_geofence_llt_ids;
 // active_geofence_llt_ids.insert(gf->id_);

  cav_msgs::CheckActiveGeofence check = wmb.checkActiveGeofenceLogic(input_msg);
  ASSERT_GE(check.distance_to_next_geofence, 0);
  EXPECT_TRUE(check.type > 0);
  EXPECT_TRUE(check.is_on_active_geofence);

  activated = false;
  ros::Time::setNow(ros::Time(3.2));  // Geofences deactivate now
  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, curr_id_hashed, last_inactive_gf));
  ASSERT_EQ(2, map_update_call_count.load());


}


}  // namespace carma_wm_ctrl
