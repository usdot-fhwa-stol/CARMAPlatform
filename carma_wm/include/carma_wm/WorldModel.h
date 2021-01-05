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

#include <exception>
#include <memory>
#include <tuple>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/utility/Optional.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include "TrackPos.h"

namespace carma_wm
{
// Helpful using declarations which are not defined by lanelet::routing module
using LaneletRoutePtr = std::shared_ptr<lanelet::routing::Route>;
using LaneletRouteConstPtr = std::shared_ptr<const lanelet::routing::Route>;
using LaneletRouteUPtr = std::unique_ptr<lanelet::routing::Route>;
using LaneletRouteUConstPtr = std::unique_ptr<const lanelet::routing::Route>;

using LaneletRoutingGraphPtr = std::shared_ptr<lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphConstPtr = std::shared_ptr<const lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphUPtr = std::unique_ptr<lanelet::routing::RoutingGraph>;
using LaneletRoutingGraphConstUPtr = std::unique_ptr<const lanelet::routing::RoutingGraph>;

using TrafficRulesConstPtr = std::shared_ptr<const lanelet::traffic_rules::TrafficRules>;
using TrafficRulesUConstPtr = std::unique_ptr<const lanelet::traffic_rules::TrafficRules>;

// Helpful enums for dividing lane into sections of interest.
enum LaneSection{LANE_AHEAD, LANE_BEHIND, LANE_FULL};

/*! \brief An interface which provides read access to the semantic map and route.
 *         This class is not thread safe. All units of distance are in meters
 *
 *  Utility functions are provided by this interface for functionality not present in the lanelet2 library such as
 *  computing downtrack and crosstrack distances or road curvatures.
 */
class WorldModel
{
public:
  /**
   * @brief Virtual destructor to ensure delete safety for pointers to implementing classes
   *
   */
  virtual ~WorldModel(){};

  /*! \brief Returns a pair of TrackPos, computed in 2d, of the provided area relative to the current route.
   *        The distance is based on the Area vertex with the smallest and largest downtrack distance
   *        This method overload is the most expensive of the routeTrackPos methods
   *
   * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
   * important to consider that when using route related functions.
   *
   * \param area The lanelet2 area which will have its distance computed
   *
   * \throws std::invalid_argument If the route is not yet loaded or the area does not contain vertices
   *
   * \return A pair where the first element contains the TrackPos area's earliest downtrack vertex and the second
   * element contains the area's latest downtrack point
   */
  virtual std::pair<TrackPos, TrackPos> routeTrackPos(const lanelet::ConstArea& area) const = 0;

  /*! \brief Returns the TrackPos, computed in 2d, of the provided lanelet relative to the current route.
   *        The distance is based on the first point in the lanelet centerline
   *
   * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
   * important to consider that when using route related functions.
   *
   * \param lanelet The lanelet2 lanelet which will have its distance computed
   *
   * \throws std::invalid_argument If the route is not yet loaded or the lanelet centerline cannot be found
   *
   * \return The TrackPos containing the downtrack and crosstrack location of the lanelet's first centerline point
   */
  virtual TrackPos routeTrackPos(const lanelet::ConstLanelet& lanelet) const = 0;

  /*! \brief Returns the TrackPos, computed in 2d, of the provided point relative to the current route
   *
   * NOTE: The route definition used in this class contains discontinuities in the reference line at lane changes. It is
   * important to consider that when using route related functions.
   *
   * \param point The lanelet2 point which will have its distance computed
   *
   * \throws std::invalid_argument If the route is not yet loaded
   *
   * \return The TrackPos of the point
   */
  virtual TrackPos routeTrackPos(const lanelet::BasicPoint2d& point) const = 0;

  /*! \brief Returns a list of lanelets which are part of the route and whose downtrack bounds exist within the provided
   * start and end distances The bounds are included so areas which end exactly at start or start exactly at end are
   * included
   *
   * \param start The starting downtrack for the query
   * \param end The ending downtrack for the query
   * \param shortest_path_only If true the lanelets returned will be part of the route shortest path
   *
   * \throws std::invalid_argument If the route is not yet loaded or if start >= end
   *
   * \return A list of lanelets which contain regions that lie between start and end along the route. This function will
   * not return lanelets which are not part of the route
   */
  virtual std::vector<lanelet::ConstLanelet> getLaneletsBetween(double start, double end, bool shortest_path_only = false) const = 0;

  /*! \brief Get a pointer to the current map. If the underlying map has changed the pointer will also need to be
   * reacquired
   *
   * \return Shared pointer to underlying lanelet map. Pointer will return false on boolean check if no map is loaded
   */
  virtual lanelet::LaneletMapConstPtr getMap() const = 0;

  /*! \brief Get a pointer to the current route. If the underlying route has changed the pointer will also need to be
   * reacquired
   *
   * \return Shared pointer to underlying lanelet route. Pointer will return false on boolean check if no route or map
   * is loaded
   */
  virtual LaneletRouteConstPtr getRoute() const = 0;

  /*! \brief Get a pointer to the routing graph for the current map. If the underlying map has changed the pointer will
   * also need to be reacquired
   *
   * \return Shared pointer to underlying lanelet route graph. Pointer will return false on boolean check if no map is
   * loaded
   */
  virtual LaneletRoutingGraphConstPtr getMapRoutingGraph() const = 0;

  /*! \brief Get most recent roadway objects - all objects on the road detected by perception stack.
   *
   * \return Vector list of RoadwayObstacle which are lanelet compatible. Empty vector if no object found.
   */
  virtual std::vector<cav_msgs::RoadwayObstacle> getRoadwayObjects() const = 0;

  /*! \brief Get a pointer to the traffic rules object used internally by the world model and considered the carma
   * system default
   *
   * \param participant The lanelet participant to return the traffic rules object for. Defaults to a generic vehicle
   *
   * \return Optional Shared pointer to an intialized traffic rules object which is used by carma. Optional is false if
   * no rule set is available for the requested participant.
   */
  virtual lanelet::Optional<TrafficRulesConstPtr>
  getTrafficRules(const std::string& participant = lanelet::Participants::Vehicle) const = 0;

  /**
   * \brief Converts an ExternalObject in a RoadwayObstacle by mapping its position onto the semantic map. Can also be
   * used to determine if the object is on the roadway
   *
   * \param object the external object to convert
   *
   * \throw std::invalid_argument if the map is not set or contains no lanelets
   *
   * \return An optional RoadwayObstacle message created from the provided object. If the external object is not on the
   * roadway then the optional will be empty.
   */
  virtual lanelet::Optional<cav_msgs::RoadwayObstacle>
  toRoadwayObstacle(const cav_msgs::ExternalObject& object) const = 0;

  /**
   * \brief Gets the a lanelet the object is currently on determined by its position on the semantic map. If it's
   * across multiple lanelets, get the closest one
   *
   * \param object the external object to get the lanelet of 
   *
   * \throw std::invalid_argument if the map is not set or contains no lanelets
   *
   * \return An optional lanelet primitive that is on the semantic map. If the external object is not on the
   * roadway then the optional will be empty.
   */

  virtual lanelet::Optional<lanelet::Lanelet> 
  getIntersectingLanelet (const cav_msgs::ExternalObject& object) const = 0;

  /**
   * \brief Gets all roadway objects currently in the same lane as the given lanelet
   *
   * \param lanelet the lanelet that is part of the continuous lane
   * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
   * 
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or if the given
   * lanelet is not on the current semantic map, or lane section input is not of the three
   *
   * \return A vector of RoadwayObstacle objects that is on the current lane. 
   * Return empty vector if there is no objects on current lane or the road
   */

  virtual std::vector<cav_msgs::RoadwayObstacle> getInLaneObjects(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const = 0;
  
  /**
   * \brief Gets Cartesian distance to the closest object on the same lane as the given point
   *
   * \param object_center the point to measure the distance from
   * 
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
   * is not on the current semantic map
   *
   * \return An optional Cartesian distance in double to the closest in lane object. Return empty if there is no objects on current lane or the road
   */
  virtual lanelet::Optional<double> distToNearestObjInLane(const lanelet::BasicPoint2d& object_center) const = 0;
  
  /**
   * \brief Gets Downtrack distance to AND copy of the closest object AHEAD on the same lane as the given point. Also returns crosstrack
   * distance relative to that object. Plus downtrack if the object is ahead along the lane, and also plus crosstrack
   * if the object is to the right relative to the reference line that crosses given object_center and is parallel to the
   * centerline of the lane.
   *
   * \param object_center the point to measure the distance from
   *
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
   * is not on the current semantic map
   *
   * \return An optional tuple of <TrackPos, cav_msgs::RoadwayObstacle> to the closest in lane object AHEAD. Return empty if there is no objects on current lane or the road
   */
  virtual lanelet::Optional<std::tuple<TrackPos,cav_msgs::RoadwayObstacle>> nearestObjectAheadInLane(const lanelet::BasicPoint2d& object_center) const = 0;

  /**
   * \brief Gets Downtrack distance to AND copy of the closest object BEHIND on the same lane as the given point. Also returns crosstrack
   * distance relative to that object. Plus downtrack if the object is ahead along the lane, and also plus crosstrack
   * if the object is to the right relative to the reference line that crosses given object_center and is parallel to the
   * centerline of the lane.
   *
   * \param object_center the point to measure the distance from
   *
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given point
   * is not on the current semantic map
   *
   * \return An optional tuple of <TrackPos, cav_msgs::RoadwayObstacle> to the closest in lane object BEHIND. Return empty if there is no objects on current lane or the road
   */
  virtual lanelet::Optional<std::tuple<TrackPos,cav_msgs::RoadwayObstacle>> nearestObjectBehindInLane(const lanelet::BasicPoint2d& object_center) const = 0;

  /**
   * \brief Gets the specified lane section achievable without lane change, sorted from the start, that includes the given lanelet 
   *
   * \param lanelet the lanelet to get the full lane of
   * \param section either of LANE_AHEAD, LANE_BEHIND, LANE_FULL each including the current lanelet
   *
   * \throw std::invalid_argument if the map is not set, contains no lanelets, or the given lanelet
   * is not on the current semantic map, or lane section input is not of the three
   *
   * \return An optional vector of ConstLanalet. Returns at least the vector of given lanelet if no other is found
   */
  virtual std::vector<lanelet::ConstLanelet> getLane(const lanelet::ConstLanelet& lanelet, const LaneSection& section = LANE_AHEAD) const = 0;

  /**
   * \brief Gets the underlying lanelet, given the cartesian point on the map
   *
   * \param point Cartesian point to check the corressponding lanelet
   * \param n     Number of lanelets to return. Default is 10. As there could be many lanelets overlapping.
   * \throw std::invalid_argument if the map is not set, contains no lanelets
   *
   * \return vector of underlying lanelet, empty vector if it is not part of any lanelet
   */
  virtual std::vector<lanelet::Lanelet> getLaneletsFromPoint(const lanelet::BasicPoint2d& point, const unsigned int n) const = 0;
};

// Helpful using declarations for carma_wm classes
using WorldModelConstPtr = std::shared_ptr<const WorldModel>;
}  // namespace carma_wm
