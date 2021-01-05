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
#include <autoware_lanelet2_msgs/MapBin.h>
#include <cav_msgs/TrafficControlRequest.h>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <ros/ros.h>

namespace carma_wm_ctrl
{
/*!
 * \brief Node which provies exposes map publication and carma_wm update logic
 *
 * The WMBroadcasterNode handles updating the lanelet2 base map and publishing the new versions to the rest of the CARMA
 * Platform ROS network. The broadcaster also provides functions for adding or removing geofences from the map and
 * notifying the rest of the system.
 *
 */
class WMBroadcasterNode
{
public:
  /**
   * @brief Constructor
   */
  WMBroadcasterNode();

  /**
   * @brief Starts the Node
   *
   * @return 0 on exit with no errors
   */
  int run();

  /**
   * @brief Callback to publish a map
   *
   * @param map_msg The map message to publish
   */
  void publishMap(const autoware_lanelet2_msgs::MapBin& map_msg);
  
  /**
   * @brief Callback to publish TrafficControlRequest Messages
   *
   * @param route_msg The TrafficControlRequest message to publish
   */
  void publishCtrlReq(const cav_msgs::TrafficControlRequest& ctrlreq_msg) const;

  /**
   * @brief Callback to publish map updates (geofences)
   *
   * @param geofence_msg The geofence message to publish
   */
  void publishMapUpdate(const autoware_lanelet2_msgs::MapBin& geofence_msg) const;

   /**
   * @brief Callback to publish active geofence information
   *
   * @param active_geof_msg The geofence information to publish
   */
  void publishActiveGeofence(const cav_msgs::CheckActiveGeofence& active_geof_msg);


private:
  ros::CARMANodeHandle cnh_;
  ros::CARMANodeHandle pnh_{"~"};

  ros::CARMANodeHandle pnh2_{"/"}; //Global Scope

  ros::Publisher map_pub_;
  ros::Publisher map_update_pub_;
  ros::Publisher control_msg_pub_;

  ros::Publisher active_pub_;

  ros::Subscriber base_map_sub_;
  
  ros::Subscriber route_callmsg_sub_;


  ros::Subscriber georef_sub_;
  ros::Subscriber geofence_sub_;
  ros::Subscriber curr_location_sub_;

  WMBroadcaster wmb_;
};
}  // namespace carma_wm_ctrl
