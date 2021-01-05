#pragma once

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

#include <autoware_lanelet2_msgs/MapBin.h>
#include <cav_msgs/Route.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_wm/TrafficControl.h>

namespace carma_wm
{
/*!
 * \brief Backend logic class for WMListener
 *
 */
class WMListenerWorker
{
public:
  /*!
   * \brief Constructor
   */
  WMListenerWorker();

  /*!
   * \brief Constructor
   */
  WorldModelConstPtr getWorldModel() const;

  /*!
   * \brief Callback for new map messages. Updates the underlying map
   *
   * \param map_msg The new map messages to generate the map from
   */
  void mapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg);

  /*!
   * \brief Callback for new map update messages (geofence). Updates the underlying map
   *
   * \param geofence_msg The new map update messages to generate the map edits from
   */
  void mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg) const;

  /*!
   * \brief Callback for route message. It is a TODO: To update function when route message spec is defined
   */
  void routeCallback(const cav_msgs::RouteConstPtr& route_msg);


  /*!
   * \brief Callback for roadway objects msg
   */
  void roadwayObjectListCallback(const cav_msgs::RoadwayObstacleList& msg);

  /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *
   * \param callback A callback function that will be triggered after the world model receives a new map update
   */
  void setMapCallback(std::function<void()> callback);

  /*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *
   * \param callback A callback function that will be triggered after the world model is updated with a new route
   */
  void setRouteCallback(std::function<void()> callback);

 /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *
   * \param config_lim A callback function that will be triggered after the world model receives a new map update
   */
  void setConfigSpeedLimit(double config_lim);

/**
 *  \brief Returns the current configured speed limit value
 * 
*/
  double getConfigSpeedLimit() const;


private:
  std::shared_ptr<CARMAWorldModel> world_model_;
  std::function<void()> map_callback_;
  std::function<void()> route_callback_;
  void newRegemUpdateHelper(lanelet::Lanelet parent_llt, lanelet::RegulatoryElement* regem) const;
  double config_speed_limit_;

};
}  // namespace carma_wm
