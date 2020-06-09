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

#include <functional>
#include <mutex>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/date_defs.hpp>
#include <boost/icl/interval_set.hpp>
#include <unordered_set>
#include "ros/ros.h"
#include <carma_wm_ctrl/GeofenceScheduler.h>

#include "MapConformer.h"

#include <cav_msgs/ControlMessage.h>
#include <std_msgs/String.h>

namespace carma_wm_ctrl
{
/*!
 * \brief Class which provies exposes map publication and carma_wm update logic
 *
 * The WMBroadcaster handles updating the lanelet2 base map and publishing the new versions to the rest of the CARMA
 * Platform ROS network. The broadcaster also provides functions for adding or removing geofences from the map and
 * notifying the rest of the system.
 *
 */
class WMBroadcaster
{
public:
  using PublishMapCallback = std::function<void(const autoware_lanelet2_msgs::MapBin&)>;

  /*!
   * \brief Constructor
   */
  WMBroadcaster(PublishMapCallback map_pub, std::unique_ptr<TimerFactory> timer_factory);

  /*!
   * \brief Callback to set the base map when it has been loaded
   *
   * \param map_msg The map message to use as the base map
   */
  void baseMapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg);

  /*!
   * \brief Callback to set the base map georeference (proj string)
   *
   * \param georef_msg Proj string that specifies the georeference of the map. 
   * It is used for transfering frames between that of geofence and that of the vehicle
   */
  void geoReferenceCallback(const std_msgs::String& geo_ref);

  /*!
   * \brief Callback to add a geofence to the map
   *
   * \param geofence_msg The ROS msg of the geofence to add. 
   */
  void geofenceCallback(const cav_msgs::ControlMessage& geofence_msg);

  /*!
   * \brief Adds a geofence to the current map
   */
  void addGeofence(Geofence& gf);

  /*!
   * \brief Removes a geofence from the current map
   */
  void removeGeofence(Geofence& gf);

  /*!
   * \brief Gets the affected lanelet or areas based on the geofence_msg
   * \param geofence_msg The ROS msg that contains proj and any point that lie on the target lanelet or area
   * \throw InvalidObjectStateError if base_map is not set or the base_map's georeference is empty
   * NOTE:Currently this function only checks lanelets and will be expanded 
   * to areas in the future.
   */
  lanelet::ConstLaneletOrAreas getAffectedLaneletOrAreas(const cav_msgs::ControlMessage& geofence_msg);

private:
  lanelet::LaneletMapPtr base_map_;
  lanelet::LaneletMapPtr current_map_;
  std::vector<lanelet::LaneletMapPtr> cached_maps_;
  std::mutex map_mutex_;
  PublishMapCallback map_pub_;
  GeofenceScheduler scheduler_;
  std::string base_map_georef_;
};
}  // namespace carma_wm_ctrl
