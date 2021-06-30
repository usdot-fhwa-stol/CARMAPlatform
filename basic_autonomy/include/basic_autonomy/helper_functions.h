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
     * \brief Returns the nearest point (in terms of cartesian 2d distance) to the provided vehicle pose in the provided l
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
      int get_nearest_point_index(const std::vector<lanelet::BasicPoint2d>& points,
                                                 const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p, veh_point);
            if (distance < min_distance)
            {
                best_index = i;
                min_distance = distance;
            }
            i++;
        }
        return best_index;
    }

    /**
     * \brief Returns the nearest point (in terms of cartesian 2d distance) to the provided vehicle pose in the provided list
     * 
     * \param points The points to evaluate
     * \param state The current vehicle state
     * 
     * \return index of nearest point in points
     */
    int get_nearest_point_index(const std::vector<PointSpeedPair>& points,
                                                 const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }
        return best_index;
    }

    /**
     * \brief Returns the nearest "less than" point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
     * NOTE: This function compares the downtrack, provided by routeTrackPos, of each points in the list to get the closest one to the given point's downtrack.
     * Therefore, it is rather costlier method than comparing cartesian distance between the points and getting the closest. This way, however, the function
     * correctly returns the end point's index if the given state, despite being valid, is farther than the given points and can technically be near any of them.
     * 
     * \param points BasicLineString2d points
     * \param target_downtrack target downtrack along the route to get index near to
     * 
     * \return index of nearest point in points
     */
    int get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points, const carma_wm::WorldModelConstPtr& wm, double target_downtrack)
    {
        int best_index = points.size() - 1;
        for(int i = 0;i < points.size(); i++){
            double downtrack = wm->routeTrackPos(points[i]).downtrack;
            if(downtrack > target_downtrack){
                //If value is negative, best index should be index 0
                best_index = std::max(0, i - 1);

                ROS_DEBUG_STREAM("get_nearest_index_by_downtrack>> Found best_idx: " << best_index<<", points[i].x(): " << points[best_index].x() << ", points[i].y(): " << points[best_index].y() << ", downtrack: "<< downtrack);
                break;
            }
        }
        ROS_DEBUG_STREAM("get_nearest_index_by_downtrack>> Found best_idx: " << best_index<<", points[i].x(): " << points[best_index].x() << ", points[i].y(): " << points[best_index].y());

        return best_index;
    }

    /**
     * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists 
     * \param points Point Speed pair to split
     * \param basic_points points vector to be filled
     * \param speeds speeds vector to be filled
     */ 
     void split_point_speed_pairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    /**
     * \brief Overload: Returns the nearest "less than" point to the provided vehicle pose in the provided list by utilizing the downtrack measured along the route
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
                                      const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.X_pos_global, state.Y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        split_point_speed_pairs(points, &basic_points, &speeds);
        return get_nearest_index_by_downtrack(basic_points, wm, ending_downtrack);
    }

    /**
     * \brief Overload: Returns the nearest "less than" point to the provided vehicle pose  in the provided list by utilizing the downtrack measured along the route
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
                                      const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d state_pos(state.X_pos_global, state.Y_pos_global);
        double ending_downtrack = wm->routeTrackPos(state_pos).downtrack;
        return get_nearest_index_by_downtrack(points, wm, ending_downtrack);
    }


}
}