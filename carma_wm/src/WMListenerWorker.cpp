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
enum class GeofenceType{ INVALID, DIGITAL_SPEED_LIMIT, PASSING_CONTROL_LINE, REGION_ACCESS_RULE, DIGITAL_MINIMUM_GAP/* ... others */ };
// helper function that return geofence type as an enum, which makes it cleaner by allowing switch statement
GeofenceType resolveGeofenceType(const std::string& rule_name)
{
  if (rule_name.compare(lanelet::PassingControlLine::RuleName) == 0) return GeofenceType::PASSING_CONTROL_LINE;
  if (rule_name.compare(lanelet::DigitalSpeedLimit::RuleName) == 0) return GeofenceType::DIGITAL_SPEED_LIMIT;
  if (rule_name.compare(lanelet::RegionAccessRule::RuleName) == 0) return GeofenceType::REGION_ACCESS_RULE;
  if (rule_name.compare(lanelet::DigitalMinimumGap::RuleName) == 0) return GeofenceType::DIGITAL_MINIMUM_GAP;
}

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

bool WMListenerWorker::checkIfReRoutingNeeded() const
{
  return rerouting_flag_;
}

void WMListenerWorker::enableUpdatesWithoutRoute()
{
  route_node_flag_=true;
}

void WMListenerWorker::mapUpdateCallback(const autoware_lanelet2_msgs::MapBinConstPtr& geofence_msg)
{

  if(geofence_msg->invalidates_route==true)
  {  
    rerouting_flag_=true;
    ROS_DEBUG_STREAM("Received notice that route has been invalidated in mapUpdateCallback");
    local_geofence_msg_ = boost::make_shared<autoware_lanelet2_msgs::MapBin>(*geofence_msg);

    if(route_node_flag_!=true)
    {
     ROS_INFO_STREAM("Route is not yet available. Therefore quueing the update");
     geofence_queue_.push (geofence_msg);
     return;
    }
  }

    while (!geofence_queue_.empty())
  {
    geofence_msg=geofence_queue_.front();
    geofence_queue_.pop();
  }

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
  // we should extract general regem to specific type of regem the geofence specifies
  
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
      newRegemUpdateHelper(parent_llt, pair.second.get());
    }
  }
  
  // set the map to set a new routing
  world_model_->setMap(world_model_->getMutableMap());

  
  ROS_INFO_STREAM("Finished Applying the Map Update with Geofence Id:" << gf_ptr->id_); 
}

/*!
  * \brief This is a helper function updates the parent_llt with specified regem. This function is needed
  *        as we need to dynamic_cast from general regem to specific type of regem based on the geofence
  * \param parent_llt The Lanelet that need to register the regem
  * \param regem lanelet::RegulatoryElement* which is the type that the serializer decodes from binary
  * NOTE: Currently this function supports digital speed limit and passing control line geofence type
  */
void WMListenerWorker::newRegemUpdateHelper(lanelet::Lanelet parent_llt, lanelet::RegulatoryElement* regem) const
{
  auto factory_pcl = lanelet::RegulatoryElementFactory::create(regem->attribute(lanelet::AttributeName::Subtype).value(),
                                                            std::const_pointer_cast<lanelet::RegulatoryElementData>(regem->constData()));
      
  // we should extract general regem to specific type of regem the geofence specifies
  switch(resolveGeofenceType(regem->attribute(lanelet::AttributeName::Subtype).value()))
  {
    case GeofenceType::PASSING_CONTROL_LINE:
    {
      lanelet::PassingControlLinePtr control_line = std::dynamic_pointer_cast<lanelet::PassingControlLine>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, control_line);
      break;
    }
    case GeofenceType::DIGITAL_SPEED_LIMIT:
    {
      lanelet::DigitalSpeedLimitPtr speed = std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, speed);
      break;
    }
    case GeofenceType::REGION_ACCESS_RULE:
    {
      lanelet::RegionAccessRulePtr rar = std::dynamic_pointer_cast<lanelet::RegionAccessRule>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, rar);
      break;
    }
    case GeofenceType::DIGITAL_MINIMUM_GAP:
    {
      lanelet::DigitalMinimumGapPtr min_gap = std::dynamic_pointer_cast<lanelet::DigitalMinimumGap>(factory_pcl);
      world_model_->getMutableMap()->update(parent_llt, min_gap);
      break;
    }
    default:
      ROS_WARN_STREAM("World Model instance received an unsupported geofence type in its map update callback!");
      break;
  }
}

void WMListenerWorker::roadwayObjectListCallback(const cav_msgs::RoadwayObstacleList& msg)
{
  // this topic publishes only the objects that are on the road
  world_model_->setRoadwayObjects(msg.roadway_obstacles);
}

void WMListenerWorker::routeCallback(const cav_msgs::RouteConstPtr& route_msg)
{
  if(rerouting_flag_==true && route_msg->is_rerouted && !route_node_flag_)
  {
    local_geofence_msg_->invalidates_route = false;
    mapUpdateCallback(local_geofence_msg_);
    rerouting_flag_ = false;
  }

  if (!world_model_->getMap()) {
    ROS_ERROR_STREAM("WMListener received a route before a map was available. Dropping route message.");
    return;
  }

  auto path = lanelet::ConstLanelets();
  for(auto id : route_msg->shortest_path_lanelet_ids)
  {
    auto ll = world_model_->getMap()->laneletLayer.get(id);
    path.push_back(ll);
  }
  if(path.empty()) return;
  auto route_opt = path.size() == 1 ? world_model_->getMapRoutingGraph()->getRoute(path.front(), path.back())
                               : world_model_->getMapRoutingGraph()->getRouteVia(path.front(), lanelet::ConstLanelets(path.begin() + 1, path.end() - 1), path.back());
  if(route_opt.is_initialized()) {
    auto ptr = std::make_shared<lanelet::routing::Route>(std::move(route_opt.get()));
    world_model_->setRoute(ptr);
  }

  world_model_->setRouteEndPoint({route_msg->end_point.x,route_msg->end_point.y,route_msg->end_point.z});

  // Call route_callback_
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

void WMListenerWorker::setConfigSpeedLimit(double config_lim)
{
  config_speed_limit_ = config_lim;
  //Function to load config_limit into CarmaWorldModel
   world_model_->setConfigSpeedLimit(config_speed_limit_);
}

double WMListenerWorker::getConfigSpeedLimit() const
{
  return config_speed_limit_;
}



}  // namespace carma_wm
