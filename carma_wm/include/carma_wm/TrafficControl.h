#pragma once

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

#include <ros/ros.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>


namespace carma_wm
{
/*! \brief This class defines an update to traffic regulations received from carma_wm_broadcaster.
           This class can be sent/received over ROS network in a binary ROS msg format.
 */
using namespace lanelet::units::literals;

class TrafficControl
{
public:
  TrafficControl(){}
  TrafficControl(boost::uuids::uuid id,
                 std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list, 
                 std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> remove_list):
                 id_(id), update_list_(update_list), remove_list_(remove_list){}  

  boost::uuids::uuid id_;  // Unique id of this geofence
  // elements needed for broadcasting to the rest of map users
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list_;
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> remove_list_;
};

/**
 * [Converts carma_wm::TrafficControl object to ROS message. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::toBinMsg]
 * @param gf_ptr [Ptr to Geofence data]
 * @param msg [converted ROS message. Only "data" field is filled]
 * NOTE: When converting the geofence object, the converter fills its relevant map update
 * fields (update_list, remove_list) to be read once received at the user
 */
void toBinMsg(std::shared_ptr<carma_wm::TrafficControl> gf_ptr, autoware_lanelet2_msgs::MapBin* msg);

/**
 * [Converts Geofence binary ROS message to carma_wm::TrafficControl object. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::fromBinMsg]
 * @param msg [ROS message for geofence]
 * @param gf_ptr [Ptr to converted Geofence object]
 * NOTE: When converting the geofence object, the converter only fills its relevant map update
 * fields (update_list, remove_list) as the ROS msg doesn't hold any other data field in the object.
 */
void fromBinMsg(const autoware_lanelet2_msgs::MapBin& msg, std::shared_ptr<carma_wm::TrafficControl> gf_ptr);


}  // namespace carma_wm


// used for converting geofence into ROS msg binary
namespace boost {
namespace serialization {

template <class Archive>
// NOLINTNEXTLINE
inline void save(Archive& ar, const carma_wm::TrafficControl& gf, unsigned int /*version*/) 
{
  std::string string_id = boost::uuids::to_string(gf.id_);
  ar << string_id;
  // convert the regems that need to be removed
  size_t remove_list_size = gf.remove_list_.size();
  ar << remove_list_size;
  for (auto pair : gf.remove_list_) ar << pair;

  // convert id, regem pairs that need to be updated
  size_t update_list_size = gf.update_list_.size();
  ar << update_list_size;
  for (auto pair : gf.update_list_) ar << pair;
}

template <class Archive>
// NOLINTNEXTLINE
inline void load(Archive& ar, carma_wm::TrafficControl& gf, unsigned int /*version*/) 
{
  boost::uuids::string_generator gen;
  std::string id;
  ar >> id;
  gf.id_ = gen(id);
  // save regems to remove
  size_t remove_list_size;
  ar >> remove_list_size;
  for (auto i = 0u; i < remove_list_size; ++i) 
  {
    std::pair<lanelet::Id, lanelet::RegulatoryElementPtr> remove_item;
    ar >> remove_item;
    gf.remove_list_.push_back(remove_item);
  }

  // save parts that need to be updated
  size_t update_list_size;
  ar >> update_list_size;
  for (auto i = 0u; i < update_list_size; ++i) 
  {
    std::pair<lanelet::Id, lanelet::RegulatoryElementPtr> update_item;
    ar >> update_item;
    gf.update_list_.push_back(update_item);
  }
}

template <typename Archive>
void serialize(Archive& ar, std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>& p, unsigned int /*version*/) 
{
  ar& p.first;
  ar& p.second;
}

} // namespace serialization
} // namespace boost

BOOST_SERIALIZATION_SPLIT_FREE(carma_wm::TrafficControl);