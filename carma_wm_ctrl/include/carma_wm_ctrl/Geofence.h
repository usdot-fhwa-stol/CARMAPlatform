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
#include <lanelet2_core/primitives/Point.h>
#include "GeofenceSchedule.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_extension/regulatory_elements/DigitalSpeedLimit.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>


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

  std::vector<GeofenceSchedule> schedules;  // The schedules this geofence operates with

  std::string proj;

  std::string type_;
  

  // TODO Add rest of the attributes provided by geofences in the future
/* following regulatory element pointer is a placeholder created with rule name 'basic_regulatory_element' to later point to 
specific type of regulatory element (such as digital speed limit, passing control line)*/

  lanelet::RegulatoryElementPtr regulatory_element_ = lanelet::RegulatoryElementFactory::create("regulatory_element", lanelet::DigitalSpeedLimit::buildData(lanelet::InvalId, 5_mph, {}, {},
                                                     { lanelet::Participants::VehicleCar }));
                                                     
  // elements needed for broadcasting to the rest of map users
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> update_list_;
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> remove_list_;
  
  // we need mutable elements saved here as they will be added back through update function which only accepts mutable objects
  std::vector<std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>> prev_regems_;
  lanelet::ConstLaneletOrAreas affected_parts_;
  
  // Helper member for PassingControlLine type regulatory geofence
  bool pcl_affects_left_ = false;
  bool pcl_affects_right_ = false;
};
}  // namespace carma_wm_ctrl