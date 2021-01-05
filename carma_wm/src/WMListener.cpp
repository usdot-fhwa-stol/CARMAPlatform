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

#include <new>
#include <carma_wm/WMListener.h>
#include "WMListenerWorker.h"

namespace carma_wm
{
  // @SONAR_STOP@
WMListener::WMListener(bool multi_thread) : worker_(std::unique_ptr<WMListenerWorker>(new WMListenerWorker)), multi_threaded_(multi_thread)
{

  ROS_DEBUG_STREAM("WMListener: Creating world model listener");

  if (multi_threaded_)
  {
    ROS_DEBUG_STREAM("WMListener: Using multi-threaded subscription");
    nh_.setCallbackQueue(&async_queue_);
  }
  map_update_sub_= nh_.subscribe("map_update", 1, &WMListener::mapUpdateCallback, this);
  map_sub_ = nh_.subscribe("semantic_map", 1, &WMListenerWorker::mapCallback, worker_.get());
  route_sub_ = nh_.subscribe("route", 1, &WMListenerWorker::routeCallback, worker_.get());
  roadway_objects_sub_ = nh_.subscribe("roadway_objects", 1, &WMListenerWorker::roadwayObjectListCallback, worker_.get());

  double cL;
  nh2_.getParam("/config_speed_limit", cL);
  setConfigSpeedLimit(cL);
  


  // Set up AsyncSpinner for multi-threaded use case
  if (multi_threaded_)
  {
    wm_spinner_ = std::unique_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1, &async_queue_));
    wm_spinner_->start();
  }
}

WMListener::~WMListener()
{
  if (multi_threaded_)
  {
    wm_spinner_->stop();
  }
}

WorldModelConstPtr WMListener::getWorldModel()
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  return worker_->getWorldModel();
}

void WMListener::mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->mapUpdateCallback(geofence_msg);
}

void WMListener::setMapCallback(std::function<void()> callback)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->setMapCallback(callback);
}

void WMListener::setRouteCallback(std::function<void()> callback)
{
  const std::lock_guard<std::mutex> lock(mw_mutex_);
  worker_->setRouteCallback(callback);
}

std::unique_lock<std::mutex> WMListener::getLock(bool pre_locked)
{
  if (pre_locked)
  {
    std::unique_lock<std::mutex> lock(mw_mutex_);  // Create movable lock
    return lock;
  }
  std::unique_lock<std::mutex> lock(mw_mutex_, std::defer_lock);  // Create movable deferred lock
  return lock;
}

void WMListener::setConfigSpeedLimit(double config_lim) const
{
  worker_->setConfigSpeedLimit(config_lim);
}

// @SONAR_START@

}  // namespace carma_wm
