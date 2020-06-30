/*
 * Copyright (C) 2019 LEIDOS.
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

#include <lanelet2_extension/utility/message_conversion.h>
#include "WMListenerWorker.h"

namespace carma_wm
{
WMListenerWorker::WMListenerWorker()
{
  world_model_.reset(new CARMAWorldModel);
}

WorldModelConstPtr WMListenerWorker::getWorldModel() const
{
  return std::static_pointer_cast<const WorldModel>(world_model_);  // Cast pointer to const variant
}

void WMListenerWorker::mapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg)
{
  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);

  world_model_->setMap(new_map);

  // Call user defined map callback
  if (map_callback_)
  {
    map_callback_();
  }
}
void WMListenerWorker::mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg) const
{
  // convert ros msg to geofence object
  auto gf_ptr = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl());
  carma_wm::fromBinMsg(*geofence_msg, gf_ptr);
  ROS_INFO_STREAM("New Map Update Received with Geofence Id:" << gf_ptr->id_);

  ROS_INFO_STREAM("Geofence id" << gf_ptr->id_ << " requests removal of size: " << gf_ptr->remove_list_.size());
  for (auto pair : gf_ptr->remove_list_)
  {
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    // we can only check by id, if the element is there
    // this is only for speed optimization, as world model here should blindly accept the map update received
    for (auto regem: parent_llt.regulatoryElements())
    {
      // we can't use the deserialized element as its data address conflicts the one in this node
      if (pair.second->id() == regem->id()) world_model_->getMutableMap()->remove(parent_llt, regem);
    }
  }

  ROS_INFO_STREAM("Geofence id" << gf_ptr->id_ << " requests update of size: " << gf_ptr->update_list_.size());
  for (auto pair : gf_ptr->update_list_)
  {
    auto parent_llt = world_model_->getMutableMap()->laneletLayer.get(pair.first);
    auto regemptr_it = world_model_->getMutableMap()->regulatoryElementLayer.find(pair.second->id());
    // if this regem is already in the map
    if (regemptr_it != world_model_->getMutableMap()->regulatoryElementLayer.end())
    {
      // again we should use the element with correct data address to be consistent
      world_model_->getMutableMap()->update(parent_llt, *regemptr_it);
    }
    else
    {
      world_model_->getMutableMap()->update(parent_llt, pair.second);
    }
  }
  
  // set the map to set a new routing
  world_model_->setMap(world_model_->getMutableMap());
  ROS_INFO_STREAM("Finished Applying the Map Update with Geofence Id:" << gf_ptr->id_);
}


void WMListenerWorker::roadwayObjectListCallback(const cav_msgs::RoadwayObstacleList& msg)
{
  // this topic publishes only the objects that are on the road
  world_model_->setRoadwayObjects(msg.roadway_obstacles);
}

void WMListenerWorker::routeCallback()
{
  // TODO Implement when route message has been defined
  // world_model_->setRoute(route_obj);
  // Call route_callback_;
  if (route_callback_)
  {
    route_callback_();
  }
}

void WMListenerWorker::setMapCallback(std::function<void()> callback)
{
  map_callback_ = callback;
}

void WMListenerWorker::setRouteCallback(std::function<void()> callback)
{
  route_callback_ = callback;
}
}  // namespace carma_wm