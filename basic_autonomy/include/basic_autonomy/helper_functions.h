#pragma once
/*
 * Copyright (C) 2021 LEIDOS.
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
#include <basic_autonomy/basic_autonomy.h>

namespace basic_autonomy
{
namespace waypoint_generation
{
    
    /**
     * \brief Returns the nearest point to the provided vehicle pose in the provided list
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
    int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                                 const cav_msgs::VehicleState& state);
    /**
     * \brief Returns the nearest point to the provided vehicle pose in the provided list
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                                 const cav_msgs::VehicleState& state);
    /**
     * \brief Returns the nearest point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
     * This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given downtrack.
     * 
     * \param points BasicLineString2d points
     * \param ending_downtrack ending downtrack along the route to get index of
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double ending_downtrack);

     /**
   * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists 
   */ 
    void split_point_speed_pairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds);
    /**
     * \brief Overload: Returns the nearest point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point's index if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points The points and speed pairs to evaluate
     * \param state The current vehicle state
     * \param wm The carma world model
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<PointSpeedPair>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const cav_msgs::VehicleState& state);

    /**
     * \brief Overload: Returns the nearest point to the provided vehicle pose  in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * \param wm The carma world model
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm,
                                      const cav_msgs::VehicleState& state);

}
}