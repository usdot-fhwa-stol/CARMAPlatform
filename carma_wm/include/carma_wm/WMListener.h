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

#include <functional>
#include <mutex>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include <autoware_lanelet2_msgs/MapBin.h>

namespace carma_wm
{
class WMListenerWorker;  // Forward declaration of worker class

/*!
 * \brief Class which provies automated subscription and threading support for the world model
 *
 * Users should generally create and cache a single instance of this class within a node.
 * They can then retrieve a pointer to an initialized WorldModel object for doing queries.
 * By default this class follows the threading model of the host node, but it can operate in the background if specified
 * in the constructor. When used in a multi-threading case users can ensure threadsafe operation though usage of the
 * getLock function
 *
 * NOTE: At the moment the mechanism of route communication in ROS is not defined therefore it is a TODO: to implement
 * full route support
 */
class WMListener
{
public:
  /*!
   * \brief Constructor which can be used to specify threading behavior of this class
   *
   * By default this object follows node threading behavior (ie. waiting for ros::spin())
   * If the object is operating in multi-threaded mode a ros::AsyncSpinner is used to implement a background thread.
   *
   * \param multi_thread If true this object will subscribe using background threads. Defaults to false
   */
  WMListener(bool multi_thread = false);

  /*! \brief Destructor
   */
  ~WMListener();

  /*!
   * \brief Returns a pointer to an intialized world model instance
   *
   * \return Const pointer to a world model object
   */
  WorldModelConstPtr getWorldModel();

  /*!
   * \brief Allows user to set a callback to be triggered when a map update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param callback A callback function that will be triggered after the world model receives a new map update
   */
  void setMapCallback(std::function<void()> callback);

  /*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param callback A callback function that will be triggered after the world model is updated with a new route
   */
  void setRouteCallback(std::function<void()> callback);

  /*!
   * \brief Returns a unique_lock which can be used to lock world model updates until the user finishes a desired
   * operation. This function should be used when multiple queries are needed and this object is operating in
   * multi-threaded mode
   *
   * \param pre_locked If true the returned lock will already be locked. If false the lock will be deferred with
   * std::defer_lock Default is true
   *
   * \return std::unique_lock for thread safe access to world model data
   */
  std::unique_lock<std::mutex> getLock(bool pre_locked = true);


/*!
   * \brief Allows user to set a callback to be triggered when a route update is received
   *        NOTE: If operating in multi-threaded mode the world model will remain locked until the user function
   * completes.
   *
   * \param config_lim A function that populate the configurable speed limit value after the world model is updated with a new route
   */
  void setConfigSpeedLimit(double config_lim) const;


private:
  // Callback function that uses lock to edit the map
  void mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg);
  ros::Subscriber roadway_objects_sub_;
  ros::Subscriber map_update_sub_;
  std::unique_ptr<WMListenerWorker> worker_;
  ros::CARMANodeHandle nh_;
  ros::CallbackQueue async_queue_;
  std::unique_ptr<ros::AsyncSpinner> wm_spinner_;
  ros::Subscriber map_sub_;
  ros::Subscriber route_sub_;
  const bool multi_threaded_;
  std::mutex mw_mutex_;
 
  ros::CARMANodeHandle nh2_{"/"};
  lanelet::Velocity config_speed_limit_;


};
}  // namespace carma_wm