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
#include <carma_wm_ctrl/WMBroadcaster.h>
#include <carma_wm/Geometry.h>
#include <carma_wm/MapConformer.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <type_traits>
#include <cav_msgs/CheckActiveGeofence.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <proj.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_core/utility/Units.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_extension/utility/utilities.h>
#include <algorithm>
#include <limits>
#include <carma_wm/Geometry.h>
#include <math.h>

namespace carma_wm_ctrl
{
using std::placeholders::_1;


WMBroadcaster::WMBroadcaster(const PublishMapCallback& map_pub, const PublishMapUpdateCallback& map_update_pub, const PublishCtrlRequestCallback& control_msg_pub,
const PublishActiveGeofCallback& active_pub, std::unique_ptr<carma_utils::timers::TimerFactory> timer_factory)
  : map_pub_(map_pub), map_update_pub_(map_update_pub), control_msg_pub_(control_msg_pub), active_pub_(active_pub), scheduler_(std::move(timer_factory))
{
  scheduler_.onGeofenceActive(std::bind(&WMBroadcaster::addGeofence, this, _1));
  scheduler_.onGeofenceInactive(std::bind(&WMBroadcaster::removeGeofence, this, _1));
  std::bind(&WMBroadcaster::routeCallbackMessage, this, _1);
};

void WMBroadcaster::baseMapCallback(const autoware_lanelet2_msgs::MapBinConstPtr& map_msg)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  static bool firstCall = true;
  // This function should generally only ever be called one time so log a warning if it occurs multiple times
  if (firstCall)
  {
    firstCall = false;
    ROS_INFO("WMBroadcaster::baseMapCallback called for first time with new map message");
  }
  else
  {
    ROS_WARN("WMBroadcaster::baseMapCallback called multiple times in the same node");
  }

  lanelet::LaneletMapPtr new_map(new lanelet::LaneletMap);
  lanelet::LaneletMapPtr new_map_to_change(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map);
  lanelet::utils::conversion::fromBinMsg(*map_msg, new_map_to_change);

  base_map_ = new_map;  // Store map
  current_map_ = new_map_to_change; // broadcaster makes changes to this

  lanelet::MapConformer::ensureCompliance(base_map_, config_limit);     // Update map to ensure it complies with expectations
  lanelet::MapConformer::ensureCompliance(current_map_, config_limit);

  // Publish map
  autoware_lanelet2_msgs::MapBin compliant_map_msg;
  lanelet::utils::conversion::toBinMsg(base_map_, &compliant_map_msg);
  map_pub_(compliant_map_msg);
};

std::shared_ptr<Geofence> WMBroadcaster::geofenceFromMsg(const cav_msgs::TrafficControlMessageV01& msg_v01)
{
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
  // Get ID
  std::copy(msg_v01.id.id.begin(), msg_v01.id.id.end(), gf_ptr->id_.begin());

  // Get affected lanelet or areas by converting the georeference and querying the map using points in the geofence
  gf_ptr->affected_parts_ = getAffectedLaneletOrAreas(msg_v01);

  std::vector<lanelet::Lanelet> affected_llts;
  std::vector<lanelet::Area> affected_areas;

  // used for assigning them to the regem as parameters
  for (auto llt_or_area : gf_ptr->affected_parts_)
  {
    if (llt_or_area.isLanelet()) affected_llts.push_back(current_map_->laneletLayer.get(llt_or_area.lanelet()->id()));
    if (llt_or_area.isArea()) affected_areas.push_back(current_map_->areaLayer.get(llt_or_area.area()->id()));
  }

  // TODO: logic to determine what type of geofence goes here
  // currently only converting portion of control message that is relevant to:
  // - digital speed limit, passing control line
  lanelet::Velocity sL;
  cav_msgs::TrafficControlDetail msg_detail = msg_v01.params.detail;
 
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MAXSPEED_CHOICE) 
  {  
    //Acquire speed limit information from TafficControlDetail msg
    sL = lanelet::Velocity(msg_detail.maxspeed * lanelet::units::MPH()); 
    
    if(config_limit > 0_mph && config_limit < 80_mph)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;
    //Ensure Geofences do not provide invalid speed limit data (exceed predetermined maximum value)
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
     ROS_WARN_STREAM("Digital maximum speed limit is invalid. Value capped at max speed limit."); //Output warning message
     sL = 80_mph; //Cap the speed limit to the predetermined maximum value

    }
    if(sL < 0_mph)
    {
           ROS_WARN_STREAM("Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, { lanelet::Participants::VehicleCar }));
  }
  
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::MINSPEED_CHOICE) 
  {
    //Acquire speed limit information from TafficControlDetail msg
     sL = lanelet::Velocity(msg_detail.minspeed * lanelet::units::MPH());
     if(config_limit > 0_mph && config_limit < 80_mph)//Accounting for the configured speed limit, input zero when not in use
        sL = config_limit;
    //Ensure Geofences do not provide invalid speed limit data 
    // @SONAR_STOP@
    if(sL > 80_mph )
    {
     ROS_WARN_STREAM("Digital speed limit is invalid. Value capped at max speed limit.");
     sL = 80_mph;
    }
    if(sL < 0_mph)
    {
           ROS_WARN_STREAM("Digital  speed limit is invalid. Value set to 0mph.");
      sL = 0_mph;
    }// @SONAR_START@
    gf_ptr->regulatory_element_ = std::make_shared<lanelet::DigitalSpeedLimit>(lanelet::DigitalSpeedLimit::buildData(lanelet::utils::getId(), 
                                        sL, affected_llts, affected_areas, { lanelet::Participants::VehicleCar }));
  }
  if (msg_detail.choice == cav_msgs::TrafficControlDetail::LATPERM_CHOICE || msg_detail.choice == cav_msgs::TrafficControlDetail::LATAFFINITY_CHOICE)
  {
    addPassingControlLineFromMsg(gf_ptr, msg_v01, affected_llts);
  }

  cav_msgs::TrafficControlSchedule msg_schedule = msg_v01.params.schedule;
  
  // Get schedule
  for (auto daily_schedule : msg_schedule.between)
  {
    gf_ptr->schedules.push_back(GeofenceSchedule(msg_schedule.start,  
                                 msg_schedule.end,
                                 daily_schedule.begin,     
                                 daily_schedule.duration,
                                 msg_schedule.repeat.offset,
                                 msg_schedule.repeat.span,   
                                 msg_schedule.repeat.period));
  }
  
  return gf_ptr;
}

void WMBroadcaster::addPassingControlLineFromMsg(std::shared_ptr<Geofence> gf_ptr, const cav_msgs::TrafficControlMessageV01& msg_v01, const std::vector<lanelet::Lanelet>& affected_llts) const
{
  cav_msgs::TrafficControlDetail msg_detail;
  msg_detail = msg_v01.params.detail;
  // Get affected bounds
  lanelet::LineStrings3d pcl_bounds;
  if (msg_detail.lataffinity == cav_msgs::TrafficControlDetail::LEFT)
  {
    for (auto llt : affected_llts) pcl_bounds.push_back(llt.leftBound());
    gf_ptr->pcl_affects_left_ = true;
  }
  else //right
  {
    for (auto llt : affected_llts) pcl_bounds.push_back(llt.rightBound());
    gf_ptr->pcl_affects_right_ = true;
  }

  // Get specified participants
  ros::V_string left_participants;
  ros::V_string right_participants;
  ros::V_string participants;
  for (j2735_msgs::TrafficControlVehClass participant : msg_v01.params.vclasses)
  {
    // Currently j2735_msgs::TrafficControlVehClass::RAIL is not supported
    if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::ANY)
    {
      participants = {lanelet::Participants::Vehicle, lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle};
      break;
    }
    else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::PEDESTRIAN)
    {
      participants.push_back(lanelet::Participants::Pedestrian);
    }
    else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::BICYCLE)
    {
      participants.push_back(lanelet::Participants::Bicycle);
    }
    else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::MICROMOBILE ||
              participant.vehicle_class == j2735_msgs::TrafficControlVehClass::MOTORCYCLE)
    {
      participants.push_back(lanelet::Participants::VehicleMotorcycle);
    }
    else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::BUS)
    {
      participants.push_back(lanelet::Participants::VehicleBus);
    }
    else if (participant.vehicle_class == j2735_msgs::TrafficControlVehClass::LIGHT_TRUCK_VAN ||
            participant.vehicle_class == j2735_msgs::TrafficControlVehClass::PASSENGER_CAR)
    {
      participants.push_back(lanelet::Participants::VehicleCar);
    }
    else if (8<= participant.vehicle_class && participant.vehicle_class <= 16) // Truck enum definition range from 8-16 currently
    {
      participants.push_back(lanelet::Participants::VehicleTruck);
    }
  }

  // Create the pcl depending on the allowed passing control direction, left, right, or both
  if (msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::PERMITTED ||
      msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::PASSINGONLY)
  {
    left_participants = participants; 
  }
  else if (msg_detail.latperm[0] == cav_msgs::TrafficControlDetail::EMERGENCYONLY)
  {
    left_participants.push_back(lanelet::Participants::VehicleEmergency);
  }
  if (msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::PERMITTED ||
      msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::PASSINGONLY)
  {
    right_participants = participants; 
  }
  else if (msg_detail.latperm[1] == cav_msgs::TrafficControlDetail::EMERGENCYONLY)
  {
    right_participants.push_back(lanelet::Participants::VehicleEmergency);
  }

  gf_ptr->regulatory_element_ = std::make_shared<lanelet::PassingControlLine>(lanelet::PassingControlLine::buildData(
    lanelet::utils::getId(), pcl_bounds, left_participants, right_participants));
}
// currently only supports geofence message version 1: TrafficControlMessageV01 
void WMBroadcaster::geofenceCallback(const cav_msgs::TrafficControlMessage& geofence_msg)
{
  
  std::lock_guard<std::mutex> guard(map_mutex_);
  // quickly check if the id has been added
  if (geofence_msg.choice != cav_msgs::TrafficControlMessage::TCMV01)
    return;

  boost::uuids::uuid id;
  std::copy(geofence_msg.tcmV01.id.id.begin(), geofence_msg.tcmV01.id.id.end(), id.begin());
  if (checked_geofence_ids_.find(boost::uuids::to_string(id)) != checked_geofence_ids_.end())
    return;

  // convert reqid to string check if it has been seen before
  boost::array<uint8_t, 16UL> req_id;
  for (auto i = 0; i < 8; i ++) req_id[i] = geofence_msg.tcmV01.reqid.id[i];
  boost::uuids::uuid uuid_id;
  std::copy(req_id.begin(),req_id.end(), uuid_id.begin());
  std::string reqid = boost::uuids::to_string(uuid_id).substr(0, 8);
  // drop if the req has never been sent
  if (generated_geofence_reqids_.find(reqid) == generated_geofence_reqids_.end())
  {
    ROS_WARN_STREAM("CARMA_WM_CTRL received a TrafficControlMessage with unknown TrafficControlRequest ID (reqid): " << reqid);
    return;
  }
    
  checked_geofence_ids_.insert(boost::uuids::to_string(id));
  auto gf_ptr = geofenceFromMsg(geofence_msg.tcmV01);
  if (gf_ptr->affected_parts_.size() == 0)
  {
    ROS_WARN_STREAM("There is no applicable component in map for the new geofence message received by WMBroadcaster with id: " << gf_ptr->id_);
    return;
  }
  scheduler_.addGeofence(gf_ptr);  // Add the geofence to the scheduler
  ROS_INFO_STREAM("New geofence message received by WMBroadcaster with id: " << gf_ptr->id_);

};

void WMBroadcaster::geoReferenceCallback(const std_msgs::String& geo_ref)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  base_map_georef_ = geo_ref.data;
}

void WMBroadcaster::setMaxLaneWidth(double max_lane_width)
{
  max_lane_width_ = max_lane_width;
}

void WMBroadcaster::setConfigSpeedLimit(double cL)
{
  /*Logic to change config_lim to Velocity value config_limit*/
  config_limit = lanelet::Velocity(cL * lanelet::units::MPH());
}

// currently only supports geofence message version 1: TrafficControlMessageV01 
lanelet::ConstLaneletOrAreas WMBroadcaster::getAffectedLaneletOrAreas(const cav_msgs::TrafficControlMessageV01& tcmV01)
{
  if (!current_map_)
  {
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));
  }
  if (base_map_georef_ == "")
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map has empty proj string loaded as georeference. Therefore, WMBroadcaster failed to\n ") +
                                          std::string("get transformation between the geofence and the map"));

  PJ* geofence_in_map_proj = proj_create_crs_to_crs(PJ_DEFAULT_CTX, tcmV01.geometry.proj.c_str(), base_map_georef_.c_str(), nullptr);
  
  // convert all geofence points into our map's frame
  std::vector<lanelet::Point3d> gf_pts;
  for (auto pt : tcmV01.geometry.nodes)
  {
    PJ_COORD c {{pt.x, pt.y, 0, 0}}; // z is not currently used
    PJ_COORD c_out;
    c_out = proj_trans(geofence_in_map_proj, PJ_FWD, c);
    gf_pts.push_back(lanelet::Point3d{current_map_->pointLayer.uniqueId(), c_out.xyz.x, c_out.xyz.y});
  }

  // Logic to detect which part is affected

  std::unordered_set<lanelet::Lanelet> affected_lanelets;
  for (int idx = 0; idx < gf_pts.size(); idx ++)
  {
    std::unordered_set<lanelet::Lanelet> possible_lanelets;
    // get nearest few nearest llts within max_lane_width_
    // which actually house this geofence_point
    auto searchFunc = [&](const lanelet::BoundingBox2d& lltBox, const lanelet::Lanelet& llt) 
    {
      bool should_stop_searching = boost::geometry::distance(gf_pts[idx].basicPoint2d(), llt.polygon2d()) > max_lane_width_;
      if (!should_stop_searching && boost::geometry::within(gf_pts[idx].basicPoint2d(), llt.polygon2d()))
      {
        possible_lanelets.insert(llt);
      }
      return should_stop_searching;
    };

    // this call updates possible_lanelets
    current_map_->laneletLayer.nearestUntil(gf_pts[idx], searchFunc);

    // among these llts, filter the ones that are on same direction as the geofence using routing
    if (idx + 1 == gf_pts.size()) // we only check this for the last gf_pt after saving everything
    {
      std::unordered_set<lanelet::Lanelet> filtered = filterSuccessorLanelets(possible_lanelets, affected_lanelets);
      affected_lanelets.insert(filtered.begin(), filtered.end());
      break;
    } 

    // check if each lines connecting end points of the llt is crossing with the line connecting current and next gf_pts
    for (auto llt: possible_lanelets)
    {
      lanelet::BasicLineString2d gf_dir_line({gf_pts[idx].basicPoint2d(), gf_pts[idx+1].basicPoint2d()});
      lanelet::BasicLineString2d llt_boundary({(llt.leftBound2d().end() -1)->basicPoint2d(), (llt.rightBound2d().end() - 1)->basicPoint2d()});
      
      // record the llts that are on the same dir
      if (boost::geometry::intersects(llt_boundary, gf_dir_line))
      {
        affected_lanelets.insert(llt);
      }
      // check condition if two geofence points are in one lanelet then check matching direction and record it also
      else if (boost::geometry::within(gf_pts[idx+1].basicPoint2d(), llt.polygon2d()) && 
              affected_lanelets.find(llt) == affected_lanelets.end())
      { 
        lanelet::BasicPoint2d median({((llt.leftBound2d().end() - 1)->basicPoint2d().x() + (llt.rightBound2d().end() - 1)->basicPoint2d().x())/2 , 
                                      ((llt.leftBound2d().end() - 1)->basicPoint2d().y() + (llt.rightBound2d().end() - 1)->basicPoint2d().y())/2});
        // turn into vectors
        Eigen::Vector2d vec_to_median(median);
        Eigen::Vector2d vec_to_gf_start(gf_pts[idx].basicPoint2d());
        Eigen::Vector2d vec_to_gf_end(gf_pts[idx + 1].basicPoint2d());

        // Get vector from start to external point
        Eigen::Vector2d start_to_median = vec_to_median - vec_to_gf_start;

        // Get vector from start to end point
        Eigen::Vector2d start_to_end = vec_to_gf_end - vec_to_gf_start;

        // Get angle between both vectors
        double interior_angle = carma_wm::geometry::getAngleBetweenVectors(start_to_median, start_to_end);
        // Save the lanelet if the direction of two points inside aligns with that of the lanelet
        if (interior_angle < M_PI_2 && interior_angle >= 0) affected_lanelets.insert(llt);
      }
    }
  }
  
  // Currently only returning lanelet, but this could be expanded to LanelerOrArea compound object 
  // by implementing non-const version of that LaneletOrArea
  lanelet::ConstLaneletOrAreas affected_parts;
  affected_parts.insert(affected_parts.end(), affected_lanelets.begin(), affected_lanelets.end());
  return affected_parts;
}

// helper function that filters successor lanelets of root_lanelets from possible_lanelets
std::unordered_set<lanelet::Lanelet> WMBroadcaster::filterSuccessorLanelets(const std::unordered_set<lanelet::Lanelet>& possible_lanelets, const std::unordered_set<lanelet::Lanelet>& root_lanelets)
{
  std::unordered_set<lanelet::Lanelet> filtered_lanelets;
  // we utilize routes to filter llts that are overlapping but not connected
  lanelet::traffic_rules::TrafficRulesUPtr traffic_rules_car = lanelet::traffic_rules::TrafficRulesFactory::create(
  lanelet::traffic_rules::CarmaUSTrafficRules::Location, lanelet::Participants::VehicleCar);
  lanelet::routing::RoutingGraphUPtr map_graph = lanelet::routing::RoutingGraph::build(*current_map_, *traffic_rules_car);
  
  // as this is the last lanelet 
  // we have to filter the llts that are only geometrically overlapping yet not connected to prev llts
  for (auto recorded_llt: root_lanelets)
  {
    for (auto following_llt: map_graph->following(recorded_llt, false))
    {
      auto mutable_llt = current_map_->laneletLayer.get(following_llt.id());
      auto it = possible_lanelets.find(mutable_llt);
      if (it != possible_lanelets.end())
      {
        filtered_lanelets.insert(mutable_llt);
      }
    }
  }
  return filtered_lanelets;
}

/*!
  * \brief This is a helper function that returns true if the provided regem is marked to be changed by the geofence as there are
  *  usually multiple passing control lines are in the lanelet.
  * \param geofence_msg The ROS msg that contains geofence information
  * \param el The LaneletOrArea that houses the regem
  * \param regem The regulatoryElement that needs to be checked
  * NOTE: Currently this function only works on lanelets. It returns true if the regem is not passingControlLine or if the pcl should be changed.
  */
bool WMBroadcaster::shouldChangeControlLine(const lanelet::ConstLaneletOrArea& el,const lanelet::RegulatoryElementConstPtr& regem, std::shared_ptr<Geofence> gf_ptr) const
{
  // should change if if the regem is not a passing control line or area, which is not supported by this logic
  if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::PassingControlLine::RuleName) != 0 || !el.isLanelet())
  {
    return true;
  }
  
  lanelet::PassingControlLinePtr pcl =  std::dynamic_pointer_cast<lanelet::PassingControlLine>(current_map_->regulatoryElementLayer.get(regem->id()));
  // if this geofence's pcl doesn't match with the lanelets current bound side, return false as we shouldn't change
  bool should_change_pcl = false;
  for (auto control_line : pcl->controlLine())
  {
    if (control_line.id() == el.lanelet()->leftBound2d().id() && gf_ptr->pcl_affects_left_ ||
    control_line.id() == el.lanelet()->rightBound2d().id() && gf_ptr->pcl_affects_right_)
    {
      should_change_pcl = true;
      break;
    }
  }
  return should_change_pcl;
}

void WMBroadcaster::addRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const
{

  // First loop is to save the relation between element and regulatory element
  // so that we can add back the old one after geofence deactivates
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (!shouldChangeControlLine(el, regem, gf_ptr)) continue;

      if (regem->attribute(lanelet::AttributeName::Subtype).value() == gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
      {
        lanelet::RegulatoryElementPtr nonconst_regem = current_map_->regulatoryElementLayer.get(regem->id());
        gf_ptr->prev_regems_.push_back(std::make_pair(el.id(), nonconst_regem));
        gf_ptr->remove_list_.push_back(std::make_pair(el.id(), nonconst_regem));

        current_map_->remove(current_map_->laneletLayer.get(el.lanelet()->id()), nonconst_regem);
      }
    }
  }


  // this loop is also kept separately because previously we assumed 
  // there was existing regem, but this handles changes to all of the elements
  for (auto el: gf_ptr->affected_parts_)
  {
    // update it with new regem
    if (gf_ptr->regulatory_element_->id() != lanelet::InvalId)
    {
      current_map_->update(current_map_->laneletLayer.get(el.id()), gf_ptr->regulatory_element_);
      gf_ptr->update_list_.push_back(std::pair<lanelet::Id, lanelet::RegulatoryElementPtr>(el.id(), gf_ptr->regulatory_element_));
    }
  }
  


}

void WMBroadcaster::addBackRegulatoryComponent(std::shared_ptr<Geofence> gf_ptr) const
{
  // First loop is to remove the relation between element and regulatory element that this geofence added initially
  
  for (auto el: gf_ptr->affected_parts_)
  {
    for (auto regem : el.regulatoryElements())
    {
      if (!shouldChangeControlLine(el, regem, gf_ptr)) continue;

      if (regem->attribute(lanelet::AttributeName::Subtype).value() ==  gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
      {
        auto nonconst_regem = current_map_->regulatoryElementLayer.get(regem->id());
        gf_ptr->remove_list_.push_back(std::make_pair(el.id(), nonconst_regem));
        current_map_->remove(current_map_->laneletLayer.get(el.lanelet()->id()), nonconst_regem);
      }
    }
  }
  
  // As this gf received is the first gf that was sent in through addGeofence,
  // we have prev speed limit information inside it to put them back
  for (auto pair : gf_ptr->prev_regems_)
  {
    if (pair.second->attribute(lanelet::AttributeName::Subtype).value() ==  gf_ptr->regulatory_element_->attribute(lanelet::AttributeName::Subtype).value())
    {
      current_map_->update(current_map_->laneletLayer.get(pair.first), pair.second);
      gf_ptr->update_list_.push_back(pair);
    } 
  }
}

void WMBroadcaster::addGeofence(std::shared_ptr<Geofence> gf_ptr)
{
   
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Adding active geofence to the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  addGeofenceHelper(gf_ptr);
  
  for (auto pair : gf_ptr->update_list_) active_geofence_llt_ids_.insert(pair.first);
  

  // Publish
  autoware_lanelet2_msgs::MapBin gf_msg;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  carma_wm::toBinMsg(send_data, &gf_msg);
  map_update_pub_(gf_msg);
  

};

void WMBroadcaster::removeGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(map_mutex_);
  ROS_INFO_STREAM("Removing inactive geofence from the map with geofence id: " << gf_ptr->id_);
  
  // Process the geofence object to populate update remove lists
  removeGeofenceHelper(gf_ptr);

  for (auto pair : gf_ptr->remove_list_) active_geofence_llt_ids_.erase(pair.first);

  // publish
  autoware_lanelet2_msgs::MapBin gf_msg_revert;
  auto send_data = std::make_shared<carma_wm::TrafficControl>(carma_wm::TrafficControl(gf_ptr->id_, gf_ptr->update_list_, gf_ptr->remove_list_));
  
  carma_wm::toBinMsg(send_data, &gf_msg_revert);
  map_update_pub_(gf_msg_revert);


};
  
void  WMBroadcaster::routeCallbackMessage(const cav_msgs::Route& route_msg)
{

 cav_msgs::TrafficControlRequest cR; 
 cR =  controlRequestFromRoute(route_msg);
 control_msg_pub_(cR);

}

cav_msgs::TrafficControlRequest WMBroadcaster::controlRequestFromRoute(const cav_msgs::Route& route_msg, std::shared_ptr<j2735_msgs::Id64b> req_id_for_testing)
{
  lanelet::ConstLanelets path; 

  if (!current_map_) 
  {
   // Return / log warning etc.
    ROS_INFO_STREAM("Value 'current_map_' does not exist.");
    throw lanelet::InvalidObjectStateError(std::string("Base lanelet map is not loaded to the WMBroadcaster"));

  }

  for(auto id : route_msg.route_path_lanelet_ids) 
  {
    auto laneLayer = current_map_->laneletLayer.get(id);
    path.push_back(laneLayer);
  }

  // update local copy
  route_path_ = path;
  
  if(path.size() == 0) throw lanelet::InvalidObjectStateError(std::string("No lanelets available in path."));

   /*logic to determine route bounds*/
  std::vector<lanelet::ConstLanelet> llt; 
  std::vector<lanelet::BoundingBox2d> pathBox; 
  double minX = std::numeric_limits<double>::max();
  double minY = std::numeric_limits<double>::max();
  double maxX = std::numeric_limits<double>::lowest();
  double maxY = std::numeric_limits<double>::lowest();

  while (path.size() != 0) //Continue until there are no more lanelet elements in path
  {
      llt.push_back(path.back()); //Add a lanelet to the vector
    

      pathBox.push_back(lanelet::geometry::boundingBox2d(llt.back())); //Create a bounding box of the added lanelet and add it to the vector


      if (pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).x() < minX)
        minX = pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).x(); //minimum x-value


      if (pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).y() < minY)
        minY = pathBox.back().corner(lanelet::BoundingBox2d::BottomLeft).y(); //minimum y-value



     if (pathBox.back().corner(lanelet::BoundingBox2d::TopRight).x() > maxX)
        maxX = pathBox.back().corner(lanelet::BoundingBox2d::TopRight).x(); //maximum x-value


     if (pathBox.back().corner(lanelet::BoundingBox2d::TopRight).y() > maxY)
        maxY = pathBox.back().corner(lanelet::BoundingBox2d::TopRight).y(); //maximum y-value


      path.pop_back(); //remove the added lanelet from path an reduce pack.size() by 1
  } //end of while loop
  

  std::string target_frame = base_map_georef_;
  if (target_frame.empty()) 
  {
   // Return / log warning etc.
    ROS_INFO_STREAM("Value 'target_frame' is empty.");
    throw lanelet::InvalidObjectStateError(std::string("Base georeference map may not be loaded to the WMBroadcaster"));

  }

  lanelet::projection::LocalFrameProjector local_projector(target_frame.c_str());
  lanelet::BasicPoint3d localPoint;

  localPoint.x()= minX;
  localPoint.y()= minY;

  lanelet::GPSPoint gpsRoute = local_projector.reverse(localPoint); //If the appropriate library is included, the reverse() function can be used to convert from local xyz to lat/lon

  cav_msgs::TrafficControlRequest cR; /*Fill the latitude value in message cB with the value of lat */
  cav_msgs::TrafficControlBounds cB; /*Fill the longitude value in message cB with the value of lon*/
  
  cB.reflat = gpsRoute.lat;
  cB.reflon = gpsRoute.lon;

  cav_msgs::OffsetPoint offsetX;
  offsetX.deltax = maxX - minX;
  cav_msgs::OffsetPoint offsetY;
  offsetY.deltay = maxY - minY;
  cB.offsets[0].deltax = minX;
  cB.offsets[0].deltay = maxY;
  cB.offsets[1].deltax = maxX;
  cB.offsets[1].deltay = minY;
  cB.offsets[2].deltax = maxX;
  cB.offsets[2].deltay = maxY;
  cB.oldest =ros::Time::now();
  
  cR.choice = cav_msgs::TrafficControlRequest::TCRV01;
  
  // create 16 byte uuid
  boost::uuids::uuid uuid_id = boost::uuids::random_generator()(); 
  // take half as string
  std::string reqid = boost::uuids::to_string(uuid_id).substr(0, 8);
  std::string req_id_test = "12345678";
  generated_geofence_reqids_.insert(req_id_test);
  generated_geofence_reqids_.insert(reqid);


  // copy to reqid array
  boost::array<uint8_t, 16UL> req_id;
  std::copy(uuid_id.begin(),uuid_id.end(), req_id.begin());
  for (auto i = 0; i < 8; i ++)
  {
    cR.tcrV01.reqid.id[i] = req_id[i];
    if (req_id_for_testing) req_id_for_testing->id[i] = req_id[i];
  }

  cR.tcrV01.bounds.push_back(cB);
  
  return cR;

}

double WMBroadcaster::distToNearestActiveGeofence(const lanelet::BasicPoint2d& curr_pos)
{
  std::lock_guard<std::mutex> guard(map_mutex_);

  if (!current_map_ || current_map_->laneletLayer.size() == 0) 
  {
    throw lanelet::InvalidObjectStateError(std::string("Lanelet map (current_map_) is not loaded to the WMBroadcaster"));
  }

  // filter only the lanelets in the route
  std::vector<lanelet::Id> active_geofence_on_route;
  for (auto llt : route_path_)
  {
    if (active_geofence_llt_ids_.find(llt.id()) != active_geofence_llt_ids_.end()) 
      active_geofence_on_route.push_back(llt.id());
  }
  
  // Get the lanelet of this point
  auto curr_lanelet = current_map_->laneletLayer.nearest(curr_pos, 1)[0]; //guaranteed to at least return 1 lanelet

  // Check if this point at least is actually within this lanelets
  if (!boost::geometry::within(curr_pos, curr_lanelet.polygon2d().basicPolygon()))
    throw std::invalid_argument("Given point is not within any lanelet");

  // get route distance (downtrack + cross_track) distances to every lanelets by their ids
  std::vector<double> route_distances;
  // and take abs of cross_track to add them to get route distance
  for (auto id: active_geofence_on_route)
  {
    carma_wm::TrackPos tp = carma_wm::geometry::trackPos(current_map_->laneletLayer.get(id), curr_pos);
    // downtrack needs to be negative for lanelet to be in front of the point, 
    // also we don't account for the lanelet that the vehicle is on
    if (tp.downtrack < 0 && id != curr_lanelet.id())
    {
      double dist = fabs(tp.downtrack) + fabs(tp.crosstrack);
      route_distances.push_back(dist);
    }
  }
  std::sort(route_distances.begin(), route_distances.end());

  if (route_distances.size() != 0 ) return route_distances[0];
  else return 0.0;

}
// helper function that detects the type of geofence and delegates
void WMBroadcaster::addGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const
{
  // resetting the information inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};

  // TODO: Logic to determine what type of geofence goes here in the future
  // currently only speedchange is available, so it is assumed that
  addRegulatoryComponent(gf_ptr);
}

// helper function that detects the type of geofence and delegates
void WMBroadcaster::removeGeofenceHelper(std::shared_ptr<Geofence> gf_ptr) const
{
  // again, TODO: Logic to determine what type of geofence goes here in the future
  // reset the info inside geofence
  gf_ptr->remove_list_ = {};
  gf_ptr->update_list_ = {};
  addBackRegulatoryComponent(gf_ptr);
  // as all changes are reverted back, we no longer need prev_regems
  gf_ptr->prev_regems_ = {};
}

void WMBroadcaster::currentLocationCallback(const geometry_msgs::PoseStamped& current_pos)
{
   cav_msgs::CheckActiveGeofence check = checkActiveGeofenceLogic(current_pos);
   active_pub_(check);//Publish

}

cav_msgs::CheckActiveGeofence WMBroadcaster::checkActiveGeofenceLogic(const geometry_msgs::PoseStamped& current_pos)
{

  if (!current_map_ || current_map_->laneletLayer.size() == 0) 
  {
    throw lanelet::InvalidObjectStateError(std::string("Lanelet map 'current_map_' is not loaded to the WMBroadcaster"));
  }

//Store current position values to be compared to geofence boundary values
  double current_pos_x = current_pos.pose.position.x;
  double current_pos_y = current_pos.pose.position.y;


lanelet::BasicPoint2d curr_pos;
  curr_pos.x() = current_pos_x;
  curr_pos.y() = current_pos_y;
  
  

  auto current_llt = current_map_->laneletLayer.nearest(curr_pos, 1)[0];
  cav_msgs::CheckActiveGeofence outgoing_geof; //message to publish
  double next_distance = 0 ; //Distance to next geofence

  
  std::vector<double> route_distances;

  if (active_geofence_llt_ids_.size() <= 0 ) 
  {
    ROS_INFO_STREAM("No active geofence llt ids are loaded to the WMBroadcaster");
    return outgoing_geof;
  }


  
  
    /* determine whether or not the vehicle's current position is within an active geofence */
     if (boost::geometry::within(curr_pos, current_llt.polygon2d().basicPolygon()))
      {         
        next_distance = distToNearestActiveGeofence(curr_pos);
        for(auto id : active_geofence_llt_ids_) 
        {
          if (id == current_llt.id())
          {
            outgoing_geof.type = 1;
            outgoing_geof.is_on_active_geofence = true;
            for (auto regem: current_llt.regulatoryElements())
              {
                  if (regem->attribute(lanelet::AttributeName::Subtype).value().compare(lanelet::DigitalSpeedLimit::RuleName) == 0)
                  {
                    lanelet::DigitalSpeedLimitPtr speed =  std::dynamic_pointer_cast<lanelet::DigitalSpeedLimit>
                    (current_map_->regulatoryElementLayer.get(regem->id()));
                   outgoing_geof.value = speed->speed_limit_.value();
                 }
              }


          }
        }
      }

      outgoing_geof.distance_to_next_geofence = next_distance;

  //end for loop
    return outgoing_geof;
  

}





}  // namespace carma_wm_ctrl



