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

#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <inlanecruising_plugin/smoothing/SplineI.h>
#include <inlanecruising_plugin/smoothing/CubicSpline.h>
#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <inlanecruising_plugin/log/log.h>
#include <carma_utils/containers/containers.h>
#include <inlanecruising_plugin/smoothing/filters.h>
#include <unordered_set>

using oss = std::ostringstream;

namespace inlanecruising_plugin
{
InLaneCruisingPlugin::InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm, InLaneCruisingPluginConfig config,
                                           PublishPluginDiscoveryCB plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
{
  plugin_discovery_msg_.name = "InLaneCruisingPlugin";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
}

bool InLaneCruisingPlugin::onSpin()
{
  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool InLaneCruisingPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
{
  ros::WallTime start_time = ros::WallTime::now(); // Start timeing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  auto points_and_target_speeds = maneuvers_to_points(req.maneuver_plan.maneuvers, current_downtrack, wm_); // Convert maneuvers to points

  ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());

  auto downsampled_points =
      carma_utils::containers::downsample_vector(points_and_target_speeds, config_.downsample_ratio);

  ROS_DEBUG_STREAM("downsample_points: " << downsampled_points.size());

  ROS_DEBUG_STREAM("PlanTrajectory");

  cav_msgs::TrajectoryPlan trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = ros::Time::now();
  trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points, req.vehicle_state); // Compute the trajectory
  trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, config_.minimum_speed);

  resp.trajectory_plan = trajectory;
  resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  ros::WallTime end_time = ros::WallTime::now(); // Planning complete

  ros::WallDuration duration = end_time - start_time;
  ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

  return true;
}

std::vector<double> InLaneCruisingPlugin::apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits)
{
  ROS_DEBUG_STREAM("Speeds list size: " << speeds.size());
  ROS_DEBUG_STREAM("SpeedLimits list size: " << speed_limits.size());

  if (speeds.size() != speed_limits.size())
  {
    throw std::invalid_argument("Speeds and speed limit lists not same size");
  }
  std::vector<double> out;
  for (size_t i = 0; i < speeds.size(); i++)
  {
    out.push_back(std::min(speeds[i], speed_limits[i]));
  }

  return out;
}

Eigen::Isometry2d InLaneCruisingPlugin::compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
{
  Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

  return carma_wm::geometry::build2dEigenTransform(p1, yaw);
}

std::vector<DiscreteCurve> InLaneCruisingPlugin::compute_sub_curves(const std::vector<PointSpeedPair>& map_points)
{
  if (map_points.size() < 2)
  {
    throw std::invalid_argument("Not enough points");
  }

  std::vector<DiscreteCurve> curves;
  DiscreteCurve curve;
  curve.frame = compute_heading_frame(map_points[0].point, map_points[1].point);
  Eigen::Isometry2d map_in_curve = curve.frame.inverse();

  for (size_t i = 0; i < map_points.size() - 1; i++)
  {
    lanelet::BasicPoint2d p1 = map_in_curve * map_points[i].point;
    lanelet::BasicPoint2d p2 = map_in_curve * map_points[i + 1].point;  // TODO Optimization to cache this value

    PointSpeedPair initial_pair;
    initial_pair.point = p1;
    initial_pair.speed = map_points[i].speed;
    curve.points.push_back(initial_pair);

    bool x_dir = (p2.x() - p1.x()) > 0;
    if (!x_dir)  // If x starts going backwards we need a new curve
    {
      // New Curve
      curves.push_back(curve);

      curve = DiscreteCurve();
      curve.frame = compute_heading_frame(map_points[i].point, map_points[i + 1].point);
      map_in_curve = curve.frame.inverse();

      PointSpeedPair pair;
      pair.point = map_in_curve * map_points[i].point;
      pair.speed = map_points[i].speed;

      curve.points.push_back(pair);  // Include first point in curve
    }
  }

  curves.push_back(curve);

  return curves;
}

std::vector<PointSpeedPair> InLaneCruisingPlugin::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,
                                                                             double time_span)
{
  std::vector<lanelet::BasicPoint2d> basic_points;
  std::vector<double> speeds;
  splitPointSpeedPairs(points, &basic_points, &speeds);

  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

  size_t time_boundary_exclusive_index =
      trajectory_utils::time_boundary_index(downtracks, speeds, config_.trajectory_time_length);

  if (time_boundary_exclusive_index == 0)
  {
    throw std::invalid_argument("No points to fit in timespan"); 
  }

  std::vector<PointSpeedPair> time_bound_points;
  time_bound_points.reserve(time_boundary_exclusive_index);

  if (time_boundary_exclusive_index == points.size())
  {
    time_bound_points.insert(time_bound_points.end(), points.begin(),
                             points.end());  // All points fit within time boundary
  }
  else
  {
    time_bound_points.insert(time_bound_points.end(), points.begin(),
                             points.begin() + time_boundary_exclusive_index - 1);  // Limit points by time boundary
  }

  return time_bound_points;
}

std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
{
  ROS_DEBUG_STREAM("VehicleState: "
                   << " x: " << state.X_pos_global << " y: " << state.Y_pos_global << " yaw: " << state.orientation
                   << " speed: " << state.longitudinal_vel);

  ROS_DEBUG_STREAM("points size: " << points.size());
  
  log::printDebugPerLine(points, &log::pointSpeedPairToStream);

  int nearest_pt_index = getNearestPointIndex(points, state);

  ROS_DEBUG_STREAM("NearestPtIndex: " << nearest_pt_index);

  std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position

  auto time_bound_points = constrain_to_time_boundary(future_points, config_.trajectory_time_length);

  ROS_DEBUG_STREAM("time_bound_points: " << time_bound_points.size());

  log::printDebugPerLine(time_bound_points, &log::pointSpeedPairToStream);

  ROS_DEBUG("Got basic points ");
  std::vector<DiscreteCurve> sub_curves = compute_sub_curves(time_bound_points);

  ROS_DEBUG_STREAM("Got sub_curves " << sub_curves.size());

  std::vector<double> final_yaw_values;
  std::vector<double> final_actual_speeds;
  std::vector<lanelet::BasicPoint2d> all_sampling_points;

  for (const auto& discreet_curve : sub_curves)
  {
    ROS_DEBUG("SubCurve");

    std::vector<double> speed_limits;
    std::vector<lanelet::BasicPoint2d> curve_points;
    splitPointSpeedPairs(discreet_curve.points, &curve_points, &speed_limits);

    std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points

    if (!fit_curve)
    {  // TODO how better to handle this case
      for (size_t i = 0; i < discreet_curve.points.size() - 1; i++)
      {
        Eigen::Isometry2d point_in_map =
            curvePointInMapTF(discreet_curve.frame, discreet_curve.points[i].point, final_yaw_values.back());
        all_sampling_points.push_back(point_in_map.translation());
        final_yaw_values.push_back(final_yaw_values.back());
        final_actual_speeds.push_back(final_actual_speeds.back());
      }
      continue;
    }

    ROS_DEBUG("Got fit");

    ROS_DEBUG_STREAM("speed_limits.size() " << speed_limits.size());

    std::vector<lanelet::BasicPoint2d> sampling_points;
    sampling_points.reserve(1 + discreet_curve.points.size() * 2);

    std::vector<double> distributed_speed_limits;
    distributed_speed_limits.reserve(1 + discreet_curve.points.size() * 2);

    double max_x = curve_points.back().x();
    double current_dist = 0;
    double step_size = config_.curve_resample_step_size;
    int current_speed_index = 0;

    while (current_dist < max_x - step_size) // Resample curve at tighter resolution
    {
      double x = current_dist;
      double y = (*fit_curve)(x);
      lanelet::BasicPoint2d p(x, y);
      sampling_points.push_back(p);

      for (size_t i = current_speed_index; i < curve_points.size(); i++)
      {
        if (curve_points[i].x() >= current_dist)
        {
          current_speed_index = i;
          break;
        }
      }

      distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
      current_dist += step_size;
    }

    log::printDebugPerLine(sampling_points, &log::basicPointToStream);

    std::vector<double> yaw_values = carma_wm::geometry::compute_tangent_orientations(sampling_points);

    std::vector<double> curvatures = carma_wm::geometry::local_circular_arc_curvatures(
        sampling_points, config_.curvature_calc_lookahead_count);  

    curvatures = smoothing::moving_average_filter(curvatures, config_.moving_average_window_size);

    log::printDoublesPerLineWithPrefix("curvatures[i]: ", curvatures);

    std::vector<double> ideal_speeds =
        trajectory_utils::constrained_speeds_for_curvatures(curvatures, config_.lateral_accel_limit);

    log::printDoublesPerLineWithPrefix("ideal_speeds: ", ideal_speeds);

    std::vector<double> actual_speeds = apply_speed_limits(ideal_speeds, distributed_speed_limits);

    log::printDoublesPerLineWithPrefix("actual_speeds: ", actual_speeds);

    log::printDoublesPerLineWithPrefix("yaw_values[i]: ", yaw_values);

    for (int i = 0; i < yaw_values.size() - 1; i++)
    {  // Drop last point

      Eigen::Isometry2d point_in_map = curvePointInMapTF(discreet_curve.frame, sampling_points[i], yaw_values[i]);
      Eigen::Rotation2Dd new_rot(point_in_map.rotation());
      final_yaw_values.push_back(new_rot.smallestAngle());
      all_sampling_points.push_back(point_in_map.translation());
    }

    final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end() - 1);

    ROS_DEBUG("Appended to final");
  }


  ROS_DEBUG("Processed all curves");

  if (all_sampling_points.size() == 0)
  {
    ROS_WARN_STREAM("No trajectory points could be generated");
    return {};
  }

  log::printDoublesPerLineWithPrefix("final_actual_speeds[i]: ", final_actual_speeds);

  log::printDoublesPerLineWithPrefix("final_yaw_values[i]: ", final_yaw_values);

  // Find Lookahead Distance based on Velocity
  double lookahead_distance = get_adaptive_lookahead(state.longitudinal_vel);

  ROS_DEBUG_STREAM("Lookahead distance at current speed: " << lookahead_distance);

  // Apply lookahead speeds
  final_actual_speeds = get_lookahead_speed(all_sampling_points, final_actual_speeds, lookahead_distance);
  
  // Add current vehicle point to front of the trajectory
  lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);
  all_sampling_points.insert(all_sampling_points.begin(),
                             cur_veh_point);  // Add current vehicle position to front of sample points

  final_actual_speeds.insert(final_actual_speeds.begin(), std::max(state.longitudinal_vel, config_.minimum_speed));

  final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

  log::printDoublesPerLineWithPrefix("pre_smoot[i]: ", final_actual_speeds);
  
  // Compute points to local downtracks
  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

  log::printDoublesPerLineWithPrefix("post_shift[i]: ", final_actual_speeds);
  
  // Apply accel limits
  final_actual_speeds = trajectory_utils::apply_accel_limits_by_distance(downtracks, final_actual_speeds,
                                                                         config_.max_accel, config_.max_accel);
  log::printDoublesPerLineWithPrefix("postAccel[i]: ", final_actual_speeds);

  
  final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, config_.moving_average_window_size);
  log::printDoublesPerLineWithPrefix("post_average[i]: ", final_actual_speeds);

  for (auto& s : final_actual_speeds)  // Limit minimum speed. TODO how to handle stopping?
  {
    s = std::max(s, config_.minimum_speed);
  }

  log::printDoublesPerLineWithPrefix("post_min_speed[i]: ", final_actual_speeds);
  // Convert speeds to times
  std::vector<double> times;
  trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

  log::printDoublesPerLineWithPrefix("times[i]: ", times);
  
  // Build trajectory points
  // TODO When more plugins are implemented that might share trajectory planning the start time will need to be based
  // off the last point in the plan if an earlier plan was provided
  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
      trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, ros::Time::now());

  return traj_points;
}

double InLaneCruisingPlugin::get_adaptive_lookahead(double velocity){
  
  // lookahead:
  // v<10kph:  5m
  // 10kph<v<50kph:  0.5*v
  // v>50kph:  25m

  double lookahead = config_.minimum_lookahead_distance;

  if (velocity < config_.minimum_lookahead_speed)
  {
    lookahead = config_.minimum_lookahead_distance;
  } 
  else if (velocity >= config_.minimum_lookahead_speed && velocity < config_.maximum_lookahead_speed)
  {
    lookahead = config_.lookahead_ratio * velocity;
  } 
  else lookahead = config_.maximum_lookahead_distance;

  return lookahead;

}

std::vector<double> InLaneCruisingPlugin::get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead){
  
  if (lookahead < config_.minimum_lookahead_distance)
  {
    throw std::invalid_argument("Invalid lookahead value");
  }

  if (speeds.size() < 1)
  {
    throw std::invalid_argument("Invalid speeds vector");
  }

  if (speeds.size() != points.size())
  {
    throw std::invalid_argument("Speeds and Points lists not same size");
  }

  std::vector<double> out;
  out.reserve(speeds.size());

  for (int i = 0; i < points.size(); i++)
  {
    int idx = i;
    double min_dist = std::numeric_limits<double>::max();
    for (int j=i+1; j < points.size(); j++){
      double dist = lanelet::geometry::distance2d(points[i],points[j]);
      if (abs(lookahead - dist) <= min_dist){
        idx = j;
        min_dist = abs(lookahead - dist);
      }
    }
    out.push_back(speeds[idx]);
  }
  
  return out;
}

Eigen::Isometry2d InLaneCruisingPlugin::curvePointInMapTF(const Eigen::Isometry2d& curve_in_map,
                                                          const lanelet::BasicPoint2d& p, double yaw) const
{
  Eigen::Rotation2Dd yaw_rot(yaw);
  Eigen::Isometry2d point_in_c = carma_wm::geometry::build2dEigenTransform(p, yaw_rot);
  Eigen::Isometry2d point_in_map = curve_in_map * point_in_c;
  return point_in_map;
}

std::vector<cav_msgs::TrajectoryPlanPoint> InLaneCruisingPlugin::trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    ros::Time startTime)
{
  if (points.size() != times.size() || points.size() != yaws.size())
  {
    throw std::invalid_argument("All input vectors must have the same size");
  }

  std::vector<cav_msgs::TrajectoryPlanPoint> traj;
  traj.reserve(points.size());

  for (int i = 0; i < points.size(); i++)
  {
    cav_msgs::TrajectoryPlanPoint tpp;
    ros::Duration relative_time(times[i]);
    tpp.target_time = startTime + relative_time;
    tpp.x = points[i].x();
    tpp.y = points[i].y();
    tpp.yaw = yaws[i];

    tpp.controller_plugin_name = "default";
    tpp.planner_plugin_name = plugin_discovery_msg_.name;

    traj.push_back(tpp);
  }

  return traj;
}

std::vector<PointSpeedPair> InLaneCruisingPlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm)
{
  std::vector<PointSpeedPair> points_and_target_speeds;
  std::unordered_set<lanelet::Id> visited_lanelets;

  bool first = true;
  ROS_DEBUG_STREAM("VehDowntrack: " << max_starting_downtrack);
  for (const auto& manuever : maneuvers)
  {
    if (manuever.type != cav_msgs::Maneuver::LANE_FOLLOWING)
    {
      throw std::invalid_argument("In-Lane Cruising does not support this maneuver type");
    }

    cav_msgs::LaneFollowingManeuver lane_following_maneuver = manuever.lane_following_maneuver;

    double starting_downtrack = lane_following_maneuver.start_dist;
    if (first)
    {
      if (starting_downtrack > max_starting_downtrack)
      {
        starting_downtrack = max_starting_downtrack;
      }
      first = false;
    }

    ROS_DEBUG_STREAM("Used downtrack: " << starting_downtrack);

    auto lanelets = wm->getLaneletsBetween(starting_downtrack, lane_following_maneuver.end_dist, true);

    ROS_DEBUG_STREAM("Maneuver");
    std::vector<lanelet::ConstLanelet> lanelets_to_add;
    for (auto l : lanelets)
    {
      ROS_DEBUG_STREAM("Lanelet ID: " << l.id());
      if (visited_lanelets.find(l.id()) == visited_lanelets.end())
      {
        lanelets_to_add.push_back(l);
        visited_lanelets.insert(l.id());
      }
    }

    lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);

    first = true;
    for (auto p : route_geometry)
    {
      if (first && points_and_target_speeds.size() != 0)
      {
        first = false;
        continue;  // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
      }
      PointSpeedPair pair;
      pair.point = p;
      pair.speed = lane_following_maneuver.end_speed;
      points_and_target_speeds.push_back(pair);
    }
  }

  return points_and_target_speeds;
}

int InLaneCruisingPlugin::getNearestPointIndex(const std::vector<PointSpeedPair>& points,
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
    ROS_DEBUG_STREAM("distance: " << distance);
    ROS_DEBUG_STREAM("p: " << p.point.x() << ", " << p.point.y());
    if (distance < min_distance)
    {
      best_index = i;
      min_distance = distance;
    }
    i++;
  }

  return best_index;
}


void InLaneCruisingPlugin::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
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

std::unique_ptr<smoothing::SplineI>
InLaneCruisingPlugin::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
{
  if (basic_points.size() < 3)
  {
    ROS_WARN_STREAM("Insufficient Spline Points");
    return nullptr;
  }

  std::unique_ptr<smoothing::SplineI> spl = std::make_unique<smoothing::CubicSpline>();
  spl->setPoints(basic_points);

  return spl;
}
}  // namespace inlanecruising_plugin
