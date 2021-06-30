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

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <carma_utils/containers/containers.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <lanelet2_core/geometry/Point.h>
#include <basic_autonomy/smoothing/SplineI.h>
#include <basic_autonomy/smoothing/BSpline.h>
#include <basic_autonomy/smoothing/filters.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>

namespace basic_autonomy
{
    namespace waypoint_generation
    {
        struct PointSpeedPair
        {
            lanelet::BasicPoint2d point;
            double speed = 0;
        };

        struct GeneralTrajConfig
        {
            std::string trajectory_type = "inlanecruising";
            int default_downsample_ratio = 36.0; // Amount to downsample input lanelet centerline data.
            int turn_downsample_ratio = 20.0;    // Amount to downsample input lanelet centerline data if the lanelet is marked as a turn
                                                 // Corresponds to saving each nth point
        };

        struct DetailedTrajConfig
        {
            double trajectory_time_length = 6.0;          // Trajectory length in seconds
            double curve_resample_step_size = 1.0;        // Curve re-sampling step size in m
            double minimum_speed = 2.2352;                // Minimum allowable speed in m/s
            double max_accel = 3;                         // Maximum allowable longitudinal acceleration in m/s^2
            double lateral_accel_limit = 2.5;             // Maximum allowable lateral acceleration m/s^2
            int speed_moving_average_window_size = 5;     // Size of the window used in the moving average filter to smooth both the speed profile
            int curvature_moving_average_window_size = 9; // Size of the window used in the moving average filter to smooth the curvature profile
                                                          // computed curvature and output speeds
            double back_distance = 20;                    // Number of meters behind the first maneuver that need to be included in points for curvature calculation
            double buffer_ending_downtrack = 20.0;        //The additional downtrack beyond requested end dist used to fit points along spline
        };

        /**
   * \brief Applies the provided speed limits to the provided speeds such that each element is capped at its corresponding speed limit if needed
   * 
   * \param speeds The speeds to limit
   * \param speed_limits The speed limits to apply. Must have the same size as speeds
   * 
   * \return The capped speed limits. Has the same size as speeds
   */
        std::vector<double> apply_speed_limits(const std::vector<double> speeds, const std::vector<double> speed_limits);

        /**
   * \brief Returns a 2D coordinate frame which is located at p1 and oriented so p2 lies on the +X axis
   * 
   * \param p1 The origin point for the frame in the parent frame
   * \param p2 A point in the parent frame that will define the +X axis relative to p1
   * 
   * \return A 2D coordinate frame transform
   */
        Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d &p1, const lanelet::BasicPoint2d &p2);

        /**
   * \brief Reduces the input points to only those points that fit within the provided time boundary
   * 
   * \param points The input point speed pairs to reduce
   * \param time_span The time span in seconds which the output points will fit within
   * 
   * \return The subset of points that fit within time_span
   */
        std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair> &points, double time_span);

        /**
   * \brief Returns the min, and its idx, from the vector of values, excluding given set of values
   * \param values vector of values
   * \param excluded set of excluded values
   * 
   * \return minimum value and its idx
   */
        std::pair<double, size_t> min_with_exclusions(const std::vector<double> &values, const std::unordered_set<size_t> &excluded);

        /**
   * \brief Applies the longitudinal acceleration limit to each point's speed
   * 
   * \param downtracks downtrack distances corresponding to each speed
   * \param curv_speeds vehicle velocity in m/s.
   * \param accel_limit vehicle longitudinal acceleration in m/s^2.
   * 
   * \return optimized speeds for each dowtrack points that satisfies longitudinal acceleration
   */
        std::vector<double> optimize_speed(const std::vector<double> &downtracks, const std::vector<double> &curv_speeds, double accel_limit);

        /**
   * \brief Method combines input points, times, orientations, and an absolute start time to form a valid carma platform trajectory
   * 
   * NOTE: All input vectors must be the same size. The output vector will take this size.
   * 
   * \param points The points in the map frame that the trajectory will follow. Units m
   * \param times The times which at the vehicle should arrive at the specified points. First point should have a value of 0. Units s
   * \param yaws The orientation the vehicle should achieve at each point. Units radians
   * \param startTime The absolute start time which will be used to update the input relative times. Units s
   * 
   * \return A list of trajectory points built from the provided inputs.
   */
        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
            const std::vector<lanelet::BasicPoint2d> &points, const std::vector<double> &times,
            const std::vector<double> &yaws, ros::Time startTime);

        /**
   * \brief Attaches back_distance length of points in front of future points
   * 
   * \param points all point speed pairs
   * \param nearest_pt_index idx of nearest point to the vehicle
   * \param future_points future points before which to attach the points
   * \param back_distance number of back distance in meters
   * 
   * \return point speed pairs with back distance length of points in front of future points
   */
        std::vector<PointSpeedPair> attach_back_points(const std::vector<PointSpeedPair> &points, const int nearest_pt_index,
                                                       std::vector<PointSpeedPair> future_points, double back_distance);

        /**
   * \brief Computes a spline based on the provided points
   * \param basic_points The points to use for fitting the spline
   * 
   * \return A spline which has been fit to the provided points
   */
        std::unique_ptr<basic_autonomy::smoothing::SplineI> compute_fit(const std::vector<lanelet::BasicPoint2d> &basic_points);

        /**
   * \brief Given the curvature fit, computes the curvature at the given step along the curve
   * \param step_along_the_curve Value in double from 0.0 (curvature start) to 1.0 (curvature end) representing where to calculate the curvature
   * \param fit_curve curvature fit
   * 
   * \return Curvature (k = 1/r, 1/meter)
   */
        double compute_curvature_at(const basic_autonomy::smoothing::SplineI &fit_curve, double step_along_the_curve);

        /**
     * \brief Converts a set of requested LANE_FOLLOWING maneuvers to point speed limit pairs. 
     * \param maneuvers The list of maneuvers to convert
     * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
     *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
     * \param wm Pointer to intialized world model for semantic map access
     * 
     * \return List of centerline points paired with speed limits
     */
        std::vector<PointSpeedPair> maneuvers_to_points_lanefollow(const std::vector<cav_msgs::Maneuver> &maneuvers,
                                                                   double max_starting_downtrack,
                                                                   const carma_wm::WorldModelConstPtr &wm,
                                                                   cav_msgs::VehicleState &ending_state_before_buffer,
                                                                   const GeneralTrajConfig &general_config,
                                                                   const DetailedTrajConfig &detailed_config);

        /**
     * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning for a Lane following maneuver.
     * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
     *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
     * \param state The current state of the vehicle
     * \param state_time The abosolute time which the provided vehicle state corresponds to
     * 
     * \return A list of trajectory points to send to the carma planning stack
     */
        std::vector<cav_msgs::TrajectoryPlanPoint>
        compose_lanefollow_trajectory_from_centerline(const std::vector<PointSpeedPair> &points, const cav_msgs::VehicleState &state,
                                                      const ros::Time &state_time, const carma_wm::WorldModelConstPtr &wm, 
                                                      const cav_msgs::VehicleState &ending_state_before_buffer, carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg,
                                                      const DetailedTrajConfig &detailed_config);

        //Functions specific to lane change
        /**
   * \brief Converts a set of requested LANE_CHANGE maneuvers to point speed limit pairs. 
   * 
   * \param maneuvers The list of maneuvers to convert
   * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
   *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
   * 
   * \param wm Pointer to intialized world model for semantic map access
   * \param maneuver_fraction_completed is a reference to double which tracks the percentage of maneuver completed
   * \param ending_state_before_buffer is a cav_msgs Vehicle State type variable which records the vehicle state before a buffer is added to the maneuver points
   * \param detail_config is a struct recording the ros param values for parameters defined within it.
   * 
   * \return List of centerline points paired with speed limits
   */
    std::vector<PointSpeedPair> maneuvers_to_points_lanechange(const std::vector<cav_msgs::Maneuver> &maneuvers,
                                                                double max_starting_downtrack,
                                                                const carma_wm::WorldModelConstPtr &wm,
                                                                const cav_msgs::VehicleState& state, double &maneuver_fraction_completed,
                                                                cav_msgs::VehicleState &ending_state_before_buffer, const DetailedTrajConfig &detailed_config);

        /**
   * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning for a Lane following maneuver.
   * 
   * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
   *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
   * \param state The current state of the vehicle
   * \param state_time The abosolute time which the provided vehicle state corresponds to
   * \param wm The carma world model object which the vehicle is operating in.
   * \param ending_state_before_buffer The vehicle state before a buffer was added to the points. Used to revert the trajectory to required distance before returning.
   * 
   * \return A list of trajectory points to send to the carma planning stack
   */

        std::vector<cav_msgs::TrajectoryPlanPoint> compose_lanechange_trajectory_from_centerline(
               const std::vector<PointSpeedPair> &points, const cav_msgs::VehicleState &state, const ros::Time &state_time,
               const carma_wm::WorldModelConstPtr &wm, const cav_msgs::VehicleState &ending_state_before_buffer,
               const DetailedTrajConfig &detailed_config);

        /**
     * \brief Creates a Lanelet2 Linestring from a vector or points along the geometry 
     * \param starting_downtrack downtrack along route where maneuver starts
     * \param ending_downtrack downtrack along route where maneuver starts
     * \param wm Pointer to intialized world model for semantic map access
     * 
     * \return Points in a path from starting downtrack to ending downtrack
     */
        std::vector<lanelet::BasicPoint2d> create_route_geom(double starting_downtrack, int starting_lane_id,
                                                             double ending_downtrack, const carma_wm::WorldModelConstPtr &wm);

        /**
     * \brief Given a start and end point, create a vector of points fit through a spline between the points (using a Spline library)
     * \param start_lanelet The lanelet from which lane change starts
     * \param end_lanelet The lanelet in which lane change ends
     * 
     * \return A linestring path from start to end fit through Spline Library
     */
        lanelet::BasicLineString2d create_lanechange_path(const lanelet::ConstLanelet &start_lanelet, const lanelet::ConstLanelet &end_lanelet);

    DetailedTrajConfig compose_detailed_trajectory_config(double trajectory_time_length,
                                                            double curve_resample_step_size,
                                                            double minimum_speed,
                                                            double max_accel,
                                                            double lateral_accel_limit,
                                                            int speed_moving_average_window_size,
                                                            int curvature_moving_average_window_size,
                                                            double back_distance,
                                                            double buffer_ending_downtrack);

    GeneralTrajConfig compose_general_trajectory_config(const std::string& trajectory_type,
                                                        int default_downsample_ratio,
                                                        int turn_downsample_ratio);

    }
}