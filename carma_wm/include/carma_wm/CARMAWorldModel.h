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

#include "carma_wm/WorldModel.h"
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include <lanelet2_core/primitives/LineString.h>
#include "IndexedDistanceMap.h"
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include "TrackPos.h"

namespace carma_wm
{
/*! \brief Class which implements the WorldModel interface. In addition this class provides write access to the world
 *         model. Write access is achieved through setters for the Map and Route and getMutableMap().
 *         NOTE: This class should NOT be used in runtime code by users and is exposed solely for use in unit tests where the WMListener class cannot be instantiated. 
 *
 *  Proper usage of this class dictates that the Map and Route object be kept in sync. For this reason normal WorldModel users should not try to construct this class directly unless in unit tests.
 *
 * NOTE: This class uses the CarmaUSTrafficRules class internally to interpret routes.
 *       So routes which are set on this model should use the getTrafficRules() method to build using the correct rule
 * set
 */
class CARMAWorldModel : public WorldModel
{
public:
  /**
   * @brief Constructor
   *
   */
  CARMAWorldModel() = default;

  /**
   * @brief Destructor as required by interface
   *
   */
  ~CARMAWorldModel() = default;

  /*! \brief Set the current map
   *
   *  \param map A shared pointer to the map which will share ownership to this object
   */
  void setMap(lanelet::LaneletMapPtr map);

  /*! \brief Set the current route. This route must match the current map for this class to function properly
   *
   *  \param route A shared pointer to the route which will share ownership to this object
   */
  void setRoute(LaneletRoutePtr route);

  /*! \brief Get a mutable version of the current map
   * 
   *  NOTE: the user must make sure to setMap() after any edit to the map and to set a valid route
   */
  lanelet::LaneletMapPtr getMutableMap() const;

  /*! \brief Update internal records of roadway objects. These objects MUST be guaranteed to be on the road. 
   * 
   * These are detected by the sensor fusion node and are passed as objects compatible with lanelet 
   */
  void setRoadwayObjects(const std::vector<cav_msgs::RoadwayObstacle>& rw_objs);

  /**
   * \brief This function is called by distanceToObjectBehindInLane or distanceToObjectAheadInLane. 
   * Gets Downtrack distance to AND copy of the closest object on the same lane as the given point. Also returns crosstrack
   * distance relative to that object. Plus downtrack if the object is ahead along the lane, and also plus crosstrack
   * if the object is to the right relative to the reference line that crosses given object_center and is parallel to the
   * centerline of the lane. 
   *
   * \param object_center the point to measure the distance from
   * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
   *
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
   * is not on the current semantic map
   *
   * \return An optional tuple of <TrackPos, cav_msgs::RoadwayObstacle> to the closest in lane object. Return empty if there is no objects on current lane or the road
   */
  lanelet::Optional<std::tuple<TrackPos,cav_msgs::RoadwayObstacle>> getNearestObjInLane(const lanelet::BasicPoint2d& object_center, const LaneSection& section = LANE_AHEAD) const;

/** \param config_lim the configurable speed limit value populated from WMListener using the config_speed_limit parameter
 * in VehicleConfigParams.yaml
*
*/
  void setConfigSpeedLimit(double config_lim);
  
  ////
  // Overrides
  ////
  std::pair<TrackPos, TrackPos> routeTrackPos(const lanelet::ConstArea& area) const override;

  TrackPos routeTrackPos(const lanelet::ConstLanelet& lanelet) const override;

  TrackPos routeTrackPos(const lanelet::BasicPoint2d& point) const override;

  std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end, bool shortest_path_only = false) const override;

  lanelet::LaneletMapConstPtr getMap() const override;

  LaneletRouteConstPtr getRoute() const override;

  LaneletRoutingGraphConstPtr getMapRoutingGraph() const override;

  lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules(const std::string& participant = lanelet::Participants::Vehicle) const override;

  std::vector<cav_msgs::RoadwayObstacle> getRoadwayObjects() const override;

  std::vector<cav_msgs::RoadwayObstacle> getInLaneObjects(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const override;

  lanelet::Optional<lanelet::Lanelet> getIntersectingLanelet (const cav_msgs::ExternalObject& object) const override;

  lanelet::Optional<cav_msgs::RoadwayObstacle> toRoadwayObstacle(const cav_msgs::ExternalObject& object) const override;

  lanelet::Optional<double> distToNearestObjInLane(const lanelet::BasicPoint2d& object_center) const override;

  lanelet::Optional<std::tuple<TrackPos,cav_msgs::RoadwayObstacle>> nearestObjectAheadInLane(const lanelet::BasicPoint2d& object_center) const override;

  lanelet::Optional<std::tuple<TrackPos,cav_msgs::RoadwayObstacle>> nearestObjectBehindInLane(const lanelet::BasicPoint2d& object_center) const override;

  std::vector<lanelet::ConstLanelet> getLane(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const override;

  std::vector<lanelet::Lanelet> getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n = 10) const override;

  


private:
  
  double config_speed_limit_;
  
  /*! \brief Helper function to compute the geometry of the route downtrack/crosstrack reference line
   *         This function should generally only be called from inside the setRoute function as it uses member variables
   * set in that function
   *
   *  Sets the shortest_path_centerlines_, shortest_path_centerlines_lengths_, and
   * shortest_path_filtered_centerline_view_ member variables
   */
  void computeDowntrackReferenceLine();

  /*! \brief Helper function to perform a deep copy of a LineString and assign new ids to all the elements. Used during
   * route centerline construction
   *
   *  \param line The linestring to be coppied
   *
   *  \return A new linestring containing unique ids for all points and the linestring itself
   */
  lanelet::LineString3d copyConstructLineString(const lanelet::ConstLineString3d& line) const;

  std::shared_ptr<lanelet::LaneletMap> semantic_map_;
  LaneletRoutePtr route_;
  LaneletRoutingGraphPtr map_routing_graph_;
  
  lanelet::LaneletMapConstUPtr shortest_path_view_;  // Map containing only lanelets along the shortest path of the
                                                     // route
  std::vector<lanelet::LineString3d> shortest_path_centerlines_;  // List of disjoint centerlines seperated by lane
                                                                  // changes along the shortest path
  IndexedDistanceMap shortest_path_distance_map_;
  lanelet::LaneletMapUPtr shortest_path_filtered_centerline_view_;  // Lanelet map view of shortest path center lines
                                                                    // only
  std::vector<cav_msgs::RoadwayObstacle> roadway_objects_; // 

  
};
}  // namespace carma_wm
