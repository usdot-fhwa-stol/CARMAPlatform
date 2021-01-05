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

#include <functional>
#include <mutex>
#include <lanelet2_core/LaneletMap.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/date_defs.hpp>
#include <boost/icl/interval_set.hpp>
#include <unordered_set>
#include "ros/ros.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <carma_wm/WMListener.h>
#include <cav_msgs/Route.h>
#include <cav_msgs/TrafficControlRequest.h>
#include <cav_msgs/TrafficControlBounds.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <geometry_msgs/PoseStamped.h>

#include <carma_wm/MapConformer.h>

#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/utility/Units.h>
#include <cav_msgs/TrafficControlMessage.h>
#include <cav_msgs/CheckActiveGeofence.h>
#include <carma_wm/TrafficControl.h>
#include <std_msgs/String.h>
#include <unordered_set>

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
  using PublishMapUpdateCallback = std::function<void(const autoware_lanelet2_msgs::MapBin&)>;
  using PublishCtrlRequestCallback = std::function<void(const cav_msgs::TrafficControlRequest&)>;
  using PublishActiveGeofCallback = std::function<void(const cav_msgs::CheckActiveGeofence&)>;


  /*!
   * \brief Constructor
   */

  WMBroadcaster(const PublishMapCallback& map_pub, const PublishMapUpdateCallback& map_update_pub, const PublishCtrlRequestCallback& control_msg_pub,
  const PublishActiveGeofCallback& active_pub, std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory);

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
   * \brief Callback to add a geofence to the map. Currently only supports version 1 TrafficControlMessage
   *
   * \param geofence_msg The ROS msg of the geofence to add. 
   */
  void geofenceCallback(const cav_msgs::TrafficControlMessage& geofence_msg);

  /*!
   * \brief Adds a geofence to the current map and publishes the ROS msg
   */
  void addGeofence(std::shared_ptr<Geofence> gf_ptr);

  /*!
   * \brief Removes a geofence from the current map and publishes the ROS msg
   */
  void removeGeofence(std::shared_ptr<Geofence> gf_ptr);
  
  /*!
  * \brief Calls controlRequestFromRoute() and publishes the TrafficControlRequest Message returned after the completed operations
  * \param route_msg The message containing route information
  */
  void routeCallbackMessage(const cav_msgs::Route& route_msg);

 /*!
  * \brief Pulls vehicle information from CARMA Cloud at startup by providing its selected route in a TrafficControlRequest message that is published after a route is selected.
  * During operation at ~10s intervals the vehicle will make another control request for the remainder of its route.
  * \param route_msg The message containing route information pulled from routeCallbackMessage()
  * \param req_id_for_testing this ptr is optional. it gives req_id for developer to test TrafficControlMessage as it needs it 
  */
  cav_msgs::TrafficControlRequest controlRequestFromRoute(const cav_msgs::Route& route_msg, std::shared_ptr<j2735_msgs::Id64b> req_id_for_testing = NULL);


  /*!
   * \brief Gets the affected lanelet or areas based on the geofence_msg
   * \param geofence_msg The ROS msg that contains proj and any point that lie on the target lanelet or area
   * \throw InvalidObjectStateError if base_map is not set or the base_map's georeference is empty
   * NOTE:Currently this function only checks lanelets and will be expanded 
   * to areas in the future.
   */
  lanelet::ConstLaneletOrAreas getAffectedLaneletOrAreas(const cav_msgs::TrafficControlMessageV01& geofence_msg);

  /*!
   * \brief Sets the max lane width in meters. Geofence points are associated to a lanelet if they are 
   *        within this distance to a lanelet as geofence points are guaranteed to apply to a single lane
   */
  void setMaxLaneWidth(double max_lane_width);

/*!
   * \brief Sets the configured speed limit. 
   */
  void setConfigSpeedLimit(double cL);
  
  /*!
   * \brief Returns geofence object from TrafficControlMessageV01 ROS Msg
   * \param geofence_msg The ROS msg that contains geofence information
   * \throw InvalidObjectStateError if base_map is not set or the base_map's georeference is empty
   * NOTE:Currently this function populates digitalSpeedLimit and passingControlLine instructions
   */
  std::shared_ptr<Geofence> geofenceFromMsg(const cav_msgs::TrafficControlMessageV01& geofence_msg);

  /*!
   * \brief Returns the route distance (downtrack or crosstrack in meters) to the nearest active geofence lanelet
   * \param curr_pos Current position in local coordinates
   * \throw InvalidObjectStateError if base_map is not set
   * \throw std::invalid_argument if curr_pos is not on the road
   * \return 0 if there is no active geofence on the vehicle's route 
   */
  double distToNearestActiveGeofence(const lanelet::BasicPoint2d& curr_pos);


  void currentLocationCallback(const geometry_msgs::PoseStamped& current_pos);
  /*!
   * \brief Returns a message indicating whether or not the vehicle is inside of an active geofence lanelet
   * \param current_pos Current position of the vehicle
   * \return 0 if vehicle is not on an active geofence 
   */
  cav_msgs::CheckActiveGeofence checkActiveGeofenceLogic(const geometry_msgs::PoseStamped& current_pos);




private:
  lanelet::ConstLanelets route_path_;
  std::unordered_set<lanelet::Id> active_geofence_llt_ids_; 
  void addRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const;
  void addBackRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const;
  void removeGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const;
  void addGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const;
  bool shouldChangeControlLine(const lanelet::ConstLaneletOrArea& el,const lanelet::RegulatoryElementConstPtr& regem, std::shared_ptr<Geofence> gf_ptr) const;
  void addPassingControlLineFromMsg(std::shared_ptr<Geofence> gf_ptr, const cav_msgs::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const; 
  std::unordered_set<lanelet::Lanelet> filterSuccessorLanelets(const std::unordered_set<lanelet::Lanelet>& possible_lanelets, const std::unordered_set<lanelet::Lanelet>& root_lanelets);
  lanelet::LaneletMapPtr base_map_;
  lanelet::LaneletMapPtr current_map_;
  lanelet::Velocity config_limit;
  std::unordered_set<std::string>  checked_geofence_ids_;
  std::unordered_set<std::string>  generated_geofence_reqids_;
  std::vector<lanelet::LaneletMapPtr> cached_maps_;
  std::mutex map_mutex_;
  PublishMapCallback map_pub_;
  PublishMapUpdateCallback map_update_pub_;
  PublishCtrlRequestCallback control_msg_pub_;
  PublishActiveGeofCallback active_pub_;
  GeofenceScheduler scheduler_;
  std::string base_map_georef_;
  double max_lane_width_;
  

};
}  // namespace carma_wm_ctrl



