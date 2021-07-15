/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "platooning_tactical_plugin/platooning_tactical_plugin.h"

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
#include <platooning_tactical_plugin/smoothing/SplineI.h>
#include <platooning_tactical_plugin/smoothing/BSpline.h>
#include <platooning_tactical_plugin/log/log.h>
#include <platooning_tactical_plugin/smoothing/filters.h>
#include <unordered_set>

using oss = std::ostringstream;

namespace platooning_tactical_plugin
{
PlatooningTacticalPlugin::PlatooningTacticalPlugin(carma_wm::WorldModelConstPtr wm, PlatooningTacticalPluginConfig config,
                                           PublishPluginDiscoveryCB plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
{
  plugin_discovery_msg_.name = "PlatooningTacticalPlugin";
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = false;
  plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
  plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
}

bool PlatooningTacticalPlugin::onSpin()
{
  plugin_discovery_publisher_(plugin_discovery_msg_);
  return true;
}

bool PlatooningTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req,
                                              cav_srvs::PlanTrajectoryResponse& resp)
{
  ros::WallTime start_time = ros::WallTime::now(); // Start timeing the execution time for planning so it can be logged

  lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
  double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

  // Only plan the trajectory for the initial LANE_FOLLOWING maneuver and any immediately sequential maneuvers of the same type
  std::vector<cav_msgs::Maneuver> maneuver_plan;
  for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++)
  {
    if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_FOLLOWING)
    {
      if (req.maneuver_plan.maneuvers[i].lane_following_maneuver.parameters.neogition_type != cav_msgs::ManeuverParameters::NO_NEGOTIATION)
      {
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
        resp.related_maneuvers.push_back(i);
      }
    }
    else
    {
      break;
    }
  }





  auto points_and_target_speeds = maneuvers_to_points(maneuver_plan, std::max((double)0, current_downtrack - config_.back_distance), wm_); // Convert maneuvers to points

  ROS_DEBUG_STREAM("points_and_target_speeds: " << points_and_target_speeds.size());

  ROS_DEBUG_STREAM("PlanTrajectory");

  cav_msgs::TrajectoryPlan original_trajectory;
  original_trajectory.header.frame_id = "map";
  original_trajectory.header.stamp = ros::Time::now();
  original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

  original_trajectory.trajectory_points = compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state, req.header.stamp); // Compute the trajectory
  original_trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, config_.minimum_speed);
  
  resp.trajectory_plan = original_trajectory;
  
  resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

  ros::WallTime end_time = ros::WallTime::now();  // Planning complete

  ros::WallDuration duration = end_time - start_time;
  ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

  return true;

}

std::vector<double> PlatooningTacticalPlugin::apply_speed_limits(const std::vector<double> speeds,
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

Eigen::Isometry2d PlatooningTacticalPlugin::compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
{
  Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

  return carma_wm::geometry::build2dEigenTransform(p1, yaw);
}

std::pair<double, size_t> PlatooningTacticalPlugin::min_with_exclusions(const std::vector<double>& values, const std::unordered_set<size_t>& excluded) const {
  double min = std::numeric_limits<double>::max();
  size_t best_idx = -1;
  for (size_t i = 0; i < values.size(); i++) {
    if (excluded.find(i) != excluded.end()) {
      continue;
    }

    if (values[i] < min) {
      min = values[i];
      best_idx = i;
    }
  }

  return std::make_pair(min, best_idx);
}

std::vector<double> PlatooningTacticalPlugin::optimize_speed(const std::vector<double>& downtracks, const std::vector<double>& curv_speeds, double accel_limit)
{
  if (downtracks.size() != curv_speeds.size())
  {
    throw std::invalid_argument("Downtracks and speeds do not have the same size");
  }

  if (accel_limit <= 0)
  {
    throw std::invalid_argument("Accel limits should be positive");
  }

  bool optimize = true;
  std::unordered_set<size_t> visited_idx;
  visited_idx.reserve(curv_speeds.size());

  std::vector<double> output = curv_speeds;

  while (optimize)
  {
    auto min_pair = min_with_exclusions(curv_speeds, visited_idx);
    size_t min_idx = std::get<1>(min_pair);
    if (min_idx == -1) {
      break;
    }

    visited_idx.insert(min_idx); // Mark this point as visited
    double v_i = std::get<0>(min_pair);
    double x_i = downtracks[min_idx];
    for (int i = min_idx - 1; i > 0; i--) { // NOTE: Do not use size_t for i type here as -- with > 0 will result in overflow
                                            //       First point's speed is left unchanged as it is current speed of the vehicle
      double v_f = curv_speeds[i];
      double dv = v_f - v_i;
      
      double x_f = downtracks[i];
      double dx = x_f - x_i;

      if(dv > 0) {
        v_f = std::min(v_f, sqrt(v_i * v_i - 2 * accel_limit * dx)); // inverting accel as we are only visiting deceleration case
        visited_idx.insert(i);
      } else if (dv < 0) {
        break;
      }
      output[i] = v_f;
      v_i = v_f;
      x_i = x_f;
    }
  }

  log::printDoublesPerLineWithPrefix("only_reverse[i]: ", output);
  
  output = trajectory_utils::apply_accel_limits_by_distance(downtracks, output, accel_limit, accel_limit);
  log::printDoublesPerLineWithPrefix("after_forward[i]: ", output);

  return output;
}


std::vector<PointSpeedPair> PlatooningTacticalPlugin::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,
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


std::vector<cav_msgs::TrajectoryPlanPoint> PlatooningTacticalPlugin::trajectory_from_points_times_orientations(
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

    // TODO: update the name
    tpp.controller_plugin_name = "PlatooningControlPlugin";
    tpp.planner_plugin_name = plugin_discovery_msg_.name;

    traj.push_back(tpp);
  }

  return traj;
}

std::vector<PointSpeedPair> PlatooningTacticalPlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
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
      throw std::invalid_argument("Platooning tactical plugin does not support this maneuver type");
    }

    if (manuever.lane_following_maneuver.parameters.neogition_type == cav_msgs::ManeuverParameters::NO_NEGOTIATION)
    {
      throw std::invalid_argument("NO_NEGOTIATION maneuver is not supported by this plugin");
    }


    // if (manuever.lane_following_maneuver.parameters.neogition_type == cav_msgs::ManeuverParameters::PLATOONING)
    // {
    //   // TODO: start and end speed are floats, sp compare them with epsilon 
    //   // if ( manuever.lane_following_maneuver.start_speed != manuever.lane_following_maneuver.end_speed)
    //   // {
    //   //   throw std::invalid_argument("Invalid Platooning Maneuver");
    //   // }
    // }

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

    auto lanelets = wm->getLaneletsBetween(starting_downtrack, lane_following_maneuver.end_dist, true, true);

    if (lanelets.empty())
    {
      ROS_ERROR_STREAM("Detected no lanelets between starting_downtrack: " << starting_downtrack  << ", and lane_following_maneuver.end_dist: " << lane_following_maneuver.end_dist);
      throw std::invalid_argument("Detected no lanelets between starting_downtrack and end_dist");
    }


    ROS_DEBUG_STREAM("Maneuver");
    lanelet::BasicLineString2d downsampled_centerline;
    downsampled_centerline.reserve(200);


    // getLaneletsBetween is inclusive lanelets between its two boundaries
    // which may return lanechange lanelets, so 
    // exclude lanechanges and plan for only the straight part
    int curr_idx = 0;
    auto following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
    lanelet::ConstLanelets straight_lanelets;

    if (lanelets.size() <= 1) // no lane change anyways if only size 1
    {
      ROS_DEBUG_STREAM("Detected one straight lanelet Id: " << lanelets[curr_idx].id());
      straight_lanelets = lanelets;
    }
    else
    {
      // skip all lanechanges until lane follow starts
      while (curr_idx + 1 < lanelets.size() && 
              std::find(following_lanelets.begin(),following_lanelets.end(), lanelets[curr_idx + 1]) == following_lanelets.end())
      {
        ROS_DEBUG_STREAM("As there were no directly following lanelets after this, skipping lanelet id: " << lanelets[curr_idx].id());
        curr_idx ++;
        following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
      }

      ROS_DEBUG_STREAM("Added lanelet Id for lane follow: " << lanelets[curr_idx].id());
      // guaranteed to have at least one "straight" lanelet (e.g the last one in the list)
      straight_lanelets.push_back(lanelets[curr_idx]);

      // add all lanelets on the straight road until next lanechange
      while (curr_idx + 1 < lanelets.size() && 
              std::find(following_lanelets.begin(),following_lanelets.end(), lanelets[curr_idx + 1]) != following_lanelets.end())
      {
        curr_idx++;
        ROS_DEBUG_STREAM("Added lanelet Id forlane follow: " << lanelets[curr_idx].id());
        straight_lanelets.push_back(lanelets[curr_idx]);
        following_lanelets = wm->getMapRoutingGraph()->following(lanelets[curr_idx]);
      }
    }

    for (auto l : straight_lanelets)
    {
      ROS_DEBUG_STREAM("Processing lanelet ID: " << l.id());
      if (visited_lanelets.find(l.id()) == visited_lanelets.end())
      {

        bool is_turn = false;
        if(l.hasAttribute("turn_direction")) {
          std::string turn_direction = l.attribute("turn_direction").value();
          is_turn = turn_direction.compare("left") == 0 || turn_direction.compare("right") == 0;
        }
        
        lanelet::BasicLineString2d centerline = l.centerline2d().basicLineString();
        lanelet::BasicLineString2d downsampled_points;
        if (is_turn) {
          downsampled_points = carma_utils::containers::downsample_vector(centerline, config_.turn_downsample_ratio);
        } else {
          downsampled_points = carma_utils::containers::downsample_vector(centerline, config_.default_downsample_ratio);
        }
        downsampled_centerline = carma_wm::geometry::concatenate_line_strings(downsampled_centerline, downsampled_points);
        visited_lanelets.insert(l.id());
      }
    }

    first = true;
    for (auto p : downsampled_centerline)
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

  // Here we are limiting the trajectory length to the given length by maneuver end dist as opposed to the end of lanelets involved.
    double starting_route_downtrack = wm_->routeTrackPos(points_and_target_speeds.front().point).downtrack;
    double ending_downtrack = maneuvers.back().lane_following_maneuver.end_dist;

    if(ending_downtrack + config_.buffer_ending_downtrack < wm_->getRouteEndTrackPos().downtrack){
      ending_downtrack = ending_downtrack + config_.buffer_ending_downtrack;
    }
    else
    {
      ending_downtrack = wm_->getRouteEndTrackPos().downtrack;
    } 
    
    size_t max_i = points_and_target_speeds.size() - 1;
    size_t unbuffered_idx = points_and_target_speeds.size() - 1;
    bool found_unbuffered_idx = false;
    double dist_accumulator = starting_route_downtrack;
    lanelet::BasicPoint2d prev_point;

  for (int i = 0; i < points_and_target_speeds.size(); i ++) {
      auto current_point = points_and_target_speeds[i].point;
      if (i == 0) {
        prev_point = current_point;
        continue;
      }

      double delta_d = lanelet::geometry::distance2d(prev_point, current_point);
      ROS_DEBUG_STREAM("Index i: " << i << ", delta_d: " << delta_d << ", dist_accumulator:" << dist_accumulator <<", current_point.x():" << current_point.x() << 
        "current_point.y():" << current_point.y());
      dist_accumulator += delta_d;
      if (dist_accumulator > maneuvers.back().lane_following_maneuver.end_dist && !found_unbuffered_idx)
      {
        unbuffered_idx = i - 1;
        ROS_DEBUG_STREAM("Found index unbuffered_idx at: " << unbuffered_idx);
        found_unbuffered_idx = true;
      }
      if (dist_accumulator > ending_downtrack) {
        max_i = i - 1;
        ROS_DEBUG_STREAM("Max_i breaking at: i: " << i << ", max_i: " << max_i);
        break;
      }
      prev_point = current_point;
    }
    ending_state_before_buffer.X_pos_global = points_and_target_speeds[unbuffered_idx].point.x();
    ending_state_before_buffer.Y_pos_global = points_and_target_speeds[unbuffered_idx].point.y();
    ROS_DEBUG_STREAM("Here ending_state_before_buffer.X_pos_global: " << ending_state_before_buffer.X_pos_global << 
      ", and Y_pos_global" << ending_state_before_buffer.Y_pos_global);

    std::vector<PointSpeedPair> constrained_points(points_and_target_speeds.begin(), points_and_target_speeds.begin() + max_i);
    return constrained_points;



////
  //   std::vector<lanelet::ConstLanelet> lanelets_to_add;
  //   for (auto l : lanelets)
  //   {
  //     ROS_DEBUG_STREAM("Lanelet ID: " << l.id());
  //     if (visited_lanelets.find(l.id()) == visited_lanelets.end())
  //     {
  //       lanelets_to_add.push_back(l);
  //       visited_lanelets.insert(l.id());
  //     }
  //   }

  //   lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);

  //   first = true;
  //   for (auto p : route_geometry)
  //   {
  //     if (first && points_and_target_speeds.size() != 0)
  //     {
  //       first = false;
  //       continue;  // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
  //     }
  //     PointSpeedPair pair;
  //     pair.point = p;
  //     pair.speed = lane_following_maneuver.end_speed;
  //     points_and_target_speeds.push_back(pair);
  //   }
  // }

  // return points_and_target_speeds;
}

int PlatooningTacticalPlugin::getNearestPointIndex(const std::vector<PointSpeedPair>& points,
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


void PlatooningTacticalPlugin::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
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
PlatooningTacticalPlugin::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
{
  if (basic_points.size() < 4)
  {
    ROS_WARN_STREAM("Insufficient Spline Points");
    return nullptr;
  }

  std::unique_ptr<smoothing::SplineI> spl = std::make_unique<smoothing::BSpline>();
  spl->setPoints(basic_points);

  return spl;
}

std::vector<cav_msgs::TrajectoryPlanPoint> PlatooningTacticalPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time)
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

  ROS_DEBUG_STREAM("time_bound_points size: " << time_bound_points.size());

  log::printDebugPerLine(time_bound_points, &log::pointSpeedPairToStream);

  ROS_DEBUG("Got basic points ");

  std::vector<PointSpeedPair> back_and_future = attach_back_points(points,nearest_pt_index,time_bound_points, config_.back_distance);

  ROS_DEBUG_STREAM("Got back_and_future points with size" <<back_and_future.size());
  log::printDebugPerLine(back_and_future, &log::pointSpeedPairToStream);

  std::vector<double> speed_limits;
  std::vector<lanelet::BasicPoint2d> curve_points;

  splitPointSpeedPairs(back_and_future, &curve_points, &speed_limits);

  std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points
  if (!fit_curve)
  {
    throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
  }

  ROS_DEBUG("Got fit");

  ROS_DEBUG_STREAM("speed_limits.size() " << speed_limits.size());

  std::vector<lanelet::BasicPoint2d> all_sampling_points;
  all_sampling_points.reserve(1 + curve_points.size() * 2);

  std::vector<double> distributed_speed_limits;
  distributed_speed_limits.reserve(1 + curve_points.size() * 2);

  // compute total length of the trajectory to get correct number of points 
  // we expect using curve_resample_step_size
  std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

  int total_step_along_curve = static_cast<int>(downtracks_raw.back() / config_.curve_resample_step_size);

  int current_speed_index = 0;
  size_t total_point_size = curve_points.size();

  double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
  double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
  std::vector<double> better_curvature;
  better_curvature.reserve(1 + curve_points.size() * 2);

  for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
  {
    lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
    
    all_sampling_points.push_back(p);
    double c = compute_curvature_at((*fit_curve), scaled_steps_along_curve);
    better_curvature.push_back(c);
    if ((double)steps_along_curve > step_threshold_for_next_speed)
    {
      step_threshold_for_next_speed += (double)total_step_along_curve / (double) total_point_size;
      current_speed_index ++;
    }
    distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
    scaled_steps_along_curve += 1.0/total_step_along_curve; //adding steps_along_curve_step_size
  }

  ROS_DEBUG_STREAM("Got sampled points with size:" << all_sampling_points.size());
  log::printDebugPerLine(all_sampling_points, &log::basicPointToStream);

  std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(all_sampling_points);

  log::printDoublesPerLineWithPrefix("raw_curvatures[i]: ", better_curvature);

  std::vector<double> curvatures = smoothing::moving_average_filter(better_curvature, config_.curvature_moving_average_window_size);

  std::vector<double> ideal_speeds =
      trajectory_utils::constrained_speeds_for_curvatures(curvatures, config_.lateral_accel_limit);
  
  log::printDoublesPerLineWithPrefix("curvatures[i]: ", curvatures);
  log::printDoublesPerLineWithPrefix("ideal_speeds: ", ideal_speeds);
  log::printDoublesPerLineWithPrefix("final_yaw_values[i]: ", final_yaw_values);

  std::vector<double> constrained_speed_limits = apply_speed_limits(ideal_speeds, distributed_speed_limits);
  log::printDoublesPerLineWithPrefix("constrained_speed_limits: ", constrained_speed_limits);

  ROS_DEBUG("Processed all points in computed fit");

  if (all_sampling_points.size() == 0)
  {
    ROS_WARN_STREAM("No trajectory points could be generated");
    return {};
  }

  // Add current vehicle point to front of the trajectory

  nearest_pt_index = get_nearest_index_by_downtrack(all_sampling_points, state);
  ROS_DEBUG_STREAM("Current state's nearest_pt_index: " << nearest_pt_index);
  ROS_DEBUG_STREAM("Curvature right now: " << better_curvature[nearest_pt_index] << ", at state x: " << state.X_pos_global << ", state y: " << state.Y_pos_global);
  ROS_DEBUG_STREAM("Corresponding to point: x: " << all_sampling_points[nearest_pt_index].x() << ", y:" << all_sampling_points[nearest_pt_index].y());

  int buffer_pt_index = get_nearest_index_by_downtrack(all_sampling_points, ending_state_before_buffer);
  ROS_DEBUG_STREAM("Ending state's index before applying buffer (buffer_pt_index): " << buffer_pt_index);
  ROS_DEBUG_STREAM("Corresponding to point: x: " << all_sampling_points[buffer_pt_index].x() << ", y:" << all_sampling_points[buffer_pt_index].y());
  
  if (nearest_pt_index + 1 >= buffer_pt_index)
  {
    ROS_WARN_STREAM("Current state is at or passed the planned end distance. Couldn't generate trajectory");
    return {};
  }

  //drop buffer points here
  std::vector<lanelet::BasicPoint2d> future_basic_points(all_sampling_points.begin() + nearest_pt_index + 1,
                                            all_sampling_points.begin()+ buffer_pt_index);  // Points in front of current vehicle position

  std::vector<double> future_speeds(constrained_speed_limits.begin() + nearest_pt_index + 1,
                                            constrained_speed_limits.begin() + buffer_pt_index);  // Points in front of current vehicle position
  std::vector<double> future_yaw(final_yaw_values.begin() + nearest_pt_index + 1,
                                            final_yaw_values.begin() + buffer_pt_index);  // Points in front of current vehicle position
  std::vector<double>  final_actual_speeds = future_speeds;
  all_sampling_points = future_basic_points;
  final_yaw_values = future_yaw;
  ROS_DEBUG_STREAM("Trimmed future points to size: " << future_basic_points.size() );

  lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);

  all_sampling_points.insert(all_sampling_points.begin(),
                             cur_veh_point);  // Add current vehicle position to front of sample points

  final_actual_speeds.insert(final_actual_speeds.begin(), state.longitudinal_vel);

  final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

  // Compute points to local downtracks
  std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

  // Apply accel limits
  final_actual_speeds = optimize_speed(downtracks, final_actual_speeds, config_.max_accel);

  log::printDoublesPerLineWithPrefix("postAccel[i]: ", final_actual_speeds);

  final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, config_.speed_moving_average_window_size);
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
  std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
      trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, state_time); 

  if (config_.publish_debug) {
    carma_debug_msgs::TrajectoryCurvatureSpeeds msg;
    msg.velocity_profile = final_actual_speeds;
    msg.relative_downtrack = downtracks;
    msg.tangent_headings = final_yaw_values;
    std::vector<double> aligned_speed_limits(constrained_speed_limits.begin() + nearest_pt_index,
                                            constrained_speed_limits.end());
    msg.speed_limits = aligned_speed_limits;
    std::vector<double> aligned_curvatures(curvatures.begin() + nearest_pt_index,
                                            curvatures.end());
    msg.curvatures = aligned_curvatures;
    msg.lat_accel_limit = config_.lateral_accel_limit;
    msg.lon_accel_limit = config_.max_accel;
    msg.starting_state = state;
    debug_msg_ = msg;
  }
  return traj_points;
}

int PlatooningTacticalPlugin::get_nearest_index_by_downtrack(const std::vector<lanelet::BasicPoint2d>& points,
                                               const cav_msgs::VehicleState& state) const
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

double PlatooningTacticalPlugin::compute_curvature_at(const smoothing::SplineI& fit_curve, double step_along_the_curve) const
{
  lanelet::BasicPoint2d f_prime_pt = fit_curve.first_deriv(step_along_the_curve);
  lanelet::BasicPoint2d f_prime_prime_pt = fit_curve.second_deriv(step_along_the_curve);
  // Convert to 3d vector to do 3d vector operations like cross.
  Eigen::Vector3d f_prime = {f_prime_pt.x(), f_prime_pt.y(), 0};
  Eigen::Vector3d f_prime_prime = {f_prime_prime_pt.x(), f_prime_prime_pt.y(), 0};
  return (f_prime.cross(f_prime_prime)).norm()/(pow(f_prime.norm(),3));
}

std::vector<PointSpeedPair> PlatooningTacticalPlugin::attach_back_points(const std::vector<PointSpeedPair>& points, 
                          const int nearest_pt_index, std::vector<PointSpeedPair> future_points, double back_distance) const
{
  std::vector<PointSpeedPair> back_and_future;
  back_and_future.reserve(points.size());
  double total_dist = 0;
  int min_i = 0;
  for (int i = nearest_pt_index; i > 0; --i) { 
    min_i = i;
    total_dist += lanelet::geometry::distance2d(points[i].point, points[i-1].point);
  
    if (total_dist > back_distance) {
      break;
    }
  }

  back_and_future.insert(back_and_future.end(), points.begin() + min_i, points.begin() + nearest_pt_index + 1);
  back_and_future.insert(back_and_future.end(), future_points.begin(), future_points.end());
  return back_and_future;
}


}  // namespace platooning_tactical_plugin
