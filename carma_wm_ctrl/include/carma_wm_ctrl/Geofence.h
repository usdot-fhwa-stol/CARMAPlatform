#pragma once
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
#include <lanelet2_core/primitives/Point.h>
#include "GeofenceSchedule.h"

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_io/io_handlers/Serialize.h>

#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>


namespace carma_wm_ctrl
{
using namespace lanelet::units::literals;
/**
 * @brief An object representing a geofence use for communications with CARMA Cloud
 *
 * TODO: This is currently a place holder class which needs to be updated based on the final geofence specification
 */
class Geofence
{
public:
  boost::uuids::uuid id_;  // Unique id of this geofence

  GeofenceSchedule schedule;  // The schedule this geofence operates with

  // TODO Add rest of the attributes provided by geofences in the future
  std::string proj;

  lanelet::DigitalSpeedLimitPtr min_speed_limit_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_kmh, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  lanelet::DigitalSpeedLimitPtr max_speed_limit_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_kmh, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
  
  // elements needed for querying and broadcasting to the rest of map users
  // we need mutable elements saved here as they will be added back through update function which only accepts mutable objects
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> prev_regems_;
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list_;
  std::vector<lanelet::RegulatoryElementPtr> remove_list_;
  lanelet::ConstLaneletOrAreas affected_parts_;
  bool is_reverse_geofence = false;
  
};

/**
 * [Converts carma_wm_ctrl::Geofence object to ROS message. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::toBinMsg]
 * @param gf_ptr [Ptr to Geofence data]
 * @param msg [converted ROS message. Only "data" field is filled]
 * NOTE: When converting the geofence object, the converter fills its relevant map update
 * fields (update_list, remove_list) to be read once at received at the user
 */
void toGeofenceBinMsg(std::shared_ptr<carma_wm_ctrl::Geofence> gf_ptr, autoware_lanelet2_msgs::MapBin* msg);

/**
 * [Converts Geofence binary ROS message to carma_wm_ctrl::Geofence object. Similar implementation to 
 * lanelet2_extension::utility::message_conversion::fromBinMsg]
 * @param msg [ROS message for geofence]
 * @param gf_ptr [Ptr to converted Geofence object]
 * NOTE: When converting the geofence object, the converter only fills its relevant map update
 * fields (update_list, remove_list) as the ROS msg doesn't hold any other data field in the object.
 */
void fromGeofenceBinMsg(const autoware_lanelet2_msgs::MapBin& msg, std::shared_ptr<carma_wm_ctrl::Geofence> gf_ptr);

}  // namespace carma_wm_ctrl


// used for converting geofence into ROS msg binary
namespace boost {
namespace serialization {

template <class Archive>
// NOLINTNEXTLINE
inline void save(Archive& ar, const carma_wm_ctrl::Geofence& gf, unsigned int /*version*/) 
{
  auto& gfnc = const_cast<carma_wm_ctrl::Geofence&>(gf);
  std::string string_id = boost::uuids::to_string(gfnc.id_);
  ar << string_id << gfnc.is_reverse_geofence;

  // convert the regems that need to be removed
  lanelet::impl::saveRegelems(ar, gfnc.remove_list_);

  // convert id, regem pairs that need to be updated
  size_t update_list_size = gfnc.update_list_.size();
  ar << update_list_size;
  for (auto pair : gf.update_list_) ar << pair;
}

template <class Archive>
// NOLINTNEXTLINE
inline void load(Archive& ar, carma_wm_ctrl::Geofence& gf, unsigned int /*version*/) 
{
  boost::uuids::string_generator gen;
  std::string id;
  ar >> id >> gf.is_reverse_geofence;
  gf.id_ = gen(id);

  // save regems to remove
  lanelet::impl::loadRegelems(ar, gf.remove_list_);

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

BOOST_SERIALIZATION_SPLIT_FREE(carma_wm_ctrl::Geofence);