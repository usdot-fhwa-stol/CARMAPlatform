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

#include <carma_wm/Geometry.h>
#include <lanelet2_core/geometry/Point.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>

namespace carma_wm
{
namespace geometry
{

constexpr double SPATIAL_EPSILON_M = 0.05;

// This safeAcos implementation is based on Stack Overflow answer: https://stackoverflow.com/questions/8489792/is-it-legal-to-take-acos-of-1-0f-or-1-0f
// Asked by SirYakalot: https://stackoverflow.com/users/956689/siryakalot
// Answered by TonyK: https://stackoverflow.com/users/428857/tonyk
// Credited in accordance with Stack Overflow's CC-BY license
double safeAcos (double x)
{
  if (x < -1.0) x = -1.0 ;
  else if (x > 1.0) x = 1.0 ;
  return std::acos(x) ;
}

void rpyFromQuaternion(const tf2::Quaternion& q, double& roll, double& pitch, double& yaw) 
{
  tf2::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);
}

void rpyFromQuaternion(const geometry_msgs::Quaternion& q_msg, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat;
  tf2::convert(q_msg, quat);
  rpyFromQuaternion(quat, roll, pitch, yaw);
}

double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2)
{
  double vec1Mag = vec1.norm();
  double vec2Mag = vec2.norm();
  if (vec1Mag == 0.0 || vec2Mag == 0.0)
  {
    return 0;
  }
  return safeAcos(vec1.dot(vec2) / (vec1Mag * vec2Mag));
}

TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                  const lanelet::BasicPoint2d& seg_end)
{
  Eigen::Vector2d vec_to_p(p);
  Eigen::Vector2d vec_to_start(seg_start);
  Eigen::Vector2d vec_to_end(seg_end);

  // Get vector from start to external point
  Eigen::Vector2d start_to_p = vec_to_p - vec_to_start;

  // Get vector from start to end point
  Eigen::Vector2d start_to_end = vec_to_end - vec_to_start;

  // Get angle between both vectors
  double interior_angle = getAngleBetweenVectors(start_to_p, start_to_end);

  // Calculate downtrack distance
  double start_to_p_mag = start_to_p.norm();
  double downtrack_dist = start_to_p_mag * std::cos(interior_angle);

  /**
   * Calculate the sign of the crosstrack distance by projecting the points to 2d
   * d = (p_x - s_x)(e_y - s_y) - (p_y - s_y)(e_x - s_x)
   * Equivalent to d = (start_to_p.x * start_to_end.y) - (start_to_p.y * start_to_end.x)
   *
   * Code below based on math equation described at
   * https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
   * Question asked by user
   * Ritvars (https://math.stackexchange.com/users/56723/ritvars)
   * and answered by user
   * Shard (https://math.stackexchange.com/users/55608/shard)
   * Attribution here is in line with Stack Overflow's Attribution policy cc-by-sa found here:
   * https://stackoverflow.blog/2009/06/25/attribution-required/
   */

  double d = (start_to_p[0] * start_to_end[1]) - (start_to_p[1] * start_to_end[0]);
  double sign = d >= 0 ? 1.0 : -1.0;  // If d is positive then the point is to the right if it is negative the point is
                                      // to the left

  double crosstrack = start_to_p_mag * std::sin(interior_angle) * sign;

  return TrackPos(downtrack_dist, crosstrack);
}

TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point)
{
  auto center_line = lanelet::utils::to2D(lanelet.centerline());

  if (center_line.numSegments() < 1)
  {
    throw std::invalid_argument("Provided lanelet has invalid centerline containing no points");
  }
  auto tuple = matchSegment(point, center_line.basicLineString());

  return std::get<0>(tuple);
}

/*! \brief Helper function to identify whether to select the preceeding or succeeding segment of a linstring point
   * that is nearest an external point when trying to find the TrackPos of the external point. Of the 3 points
   * comprising the 2 segment linestirng the external point must be closest to the midpoint for this function to be
   * valid
   *
   * ASSUMPTION: Of the 3 points comprising the 2 segment linestirng the external point is closest to the midpoint
   *
   * Meant to be called in the matchSegment method. Logic is as follows
   * If the external point is within the downtrack bounds of one segment but not the other then use the one it is within
   * If the external point is within the downtrack bounds of both segments then use the one with the smallest crosstrack
   * distance If the external point is within the downtrack bounds of both segments and has exactly equal crosstrack
   * bounds with each segment then use the preceeding segment If the external point is outside the downtrack bounds of
   * both segments then assign to the first segment as it will always have positive downtrack in this case while the
   * second segment will always have negative downtrack
   *
   * \param first_seg_trackPos The TrackPos of the external point relative to the preceeding segment
   * \param second_seg_trackPos The TrackPos of the succeeding segment
   * \param first_seg_length The length of the preceeding segment
   * \param second_seg_length The length of the succeeding segment
   *
   * \return True if the preceeding segment is the better choice. False if the succeeding segment is the better choice.
   */
  
bool selectFirstSegment(const TrackPos& first_seg_trackPos, const TrackPos& second_seg_trackPos,
                        double first_seg_length, double second_seg_length)
{
  const bool first_seg_in_downtrack =
      (0 <= first_seg_trackPos.downtrack && first_seg_trackPos.downtrack < first_seg_length);
  const bool second_seg_in_downtrack =
      (0 <= second_seg_trackPos.downtrack && second_seg_trackPos.downtrack < second_seg_length);

  if (first_seg_in_downtrack && !second_seg_in_downtrack)
  {  // If in the first segment but not the last segment

    return true;  // First segment is better
  }
  else if (second_seg_in_downtrack && !first_seg_in_downtrack)
  {
    return false;  // Second segment is better
  }
  else if (first_seg_in_downtrack && second_seg_in_downtrack)
  {
    return first_seg_trackPos.crosstrack <=
           second_seg_trackPos.crosstrack;  // Pick the first segment if the crosstrack values are equal or the first
                                            // segment is closer
  }
  else
  {  // Point lies outside the downtrack bounds of both segments (Remember According to function contract the nearest
     // point to external point is the midpoint of the two segments)
    return true;  // Choose first segment as it will always have positive downtrack in this case while the second
                  // segment will always have negative downtrack
  }
}

std::tuple<TrackPos, lanelet::BasicSegment2d> matchSegment(const lanelet::BasicPoint2d& p,
                                                           const lanelet::BasicLineString2d& line_string)
{
  if (line_string.size() < 2)
  {
    throw std::invalid_argument("Provided with linestring containing fewer than 2 points");
  }

  lanelet::BasicSegment2d best_segment =
      std::make_pair(line_string[0], line_string[1]);  // Default to starting segment if no match is found
  auto point_2d = lanelet::utils::to2D(p);

  double min_distance = lanelet::geometry::distance2d(point_2d, line_string[0]);
  size_t best_point_index = 0;
  double best_accumulated_length = 0;
  double best_last_accumulated_length = 0;
  double best_seg_length = 0;
  double best_last_seg_length = 0;
  double last_seg_length = 0;

  double accumulated_length = 0;
  double last_accumulated_length = 0;
  for (size_t i = 0; i < line_string.size(); i++)
  {  // Iterate over line string to find nearest point
    double seg_length = 0;
    if (i < line_string.size() - 1)
    {
      seg_length = lanelet::geometry::distance2d(line_string[i], line_string[i + 1]);  // Compute segment length
    }

    double distance = lanelet::geometry::distance2d(p, line_string[i]);  // Compute from current point to external point
    if (distance < min_distance)
    {  // If this distance is below minimum discovered so far then update minimum
      min_distance = distance;
      best_point_index = i;
      best_accumulated_length = accumulated_length;
      best_last_accumulated_length = last_accumulated_length;  // Record accumulated lengths to each segment
      best_seg_length = seg_length;
      best_last_seg_length = last_seg_length;
    }

    last_accumulated_length = accumulated_length;  // Update accumulated lenths
    accumulated_length += seg_length;
    last_seg_length = seg_length;
  }

  // Minimum point has been found next step is to determine which segment it should go with using the following rules.
  // If the minimum point is the first point then use the first segment
  // If the minimum point is the last point then use the last segment
  // If the minimum point is within the downtrack bounds of one segment but not the other then use the one it is within
  // If the minimum point is within the downtrack bounds of both segments then use the one with the smallest crosstrack
  // distance If the minimum point is within the downtrack bounds of both segments and has exactly equal crosstrack
  // bounds with each segment then use the first one
  TrackPos best_pos(0, 0);
  if (best_point_index == 0)
  {
    best_pos = trackPos(p, line_string[0], line_string[1]);
    best_segment = std::make_pair(line_string[0], line_string[1]);
  }
  else if (best_point_index == line_string.size() - 1)
  {
    best_pos = trackPos(p, line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
    best_pos.downtrack += best_last_accumulated_length;
    best_segment = std::make_pair(line_string[line_string.size() - 2], line_string[line_string.size() - 1]);
  }
  else
  {
    TrackPos first_seg_trackPos = trackPos(p, line_string[best_point_index - 1], line_string[best_point_index]);
    TrackPos second_seg_trackPos = trackPos(p, line_string[best_point_index], line_string[best_point_index + 1]);
    if (selectFirstSegment(first_seg_trackPos, second_seg_trackPos, best_last_seg_length, best_seg_length))
    {
      best_pos = first_seg_trackPos;
      best_pos.downtrack += best_last_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index - 1], line_string[best_point_index]);
    }
    else
    {
      best_pos = second_seg_trackPos;
      best_pos.downtrack += best_accumulated_length;
      best_segment = std::make_pair(line_string[best_point_index], line_string[best_point_index + 1]);
    }
  }

  // couldn't find a matching segment, so use the first segment within the downtrack range of.
  // Or the starting segment assuming we are before the route
  return std::make_tuple(best_pos, best_segment);
}

// NOTE: See Geometry.h header file for details on source of logic in this function
double computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2,
                        const lanelet::BasicPoint2d& p3)
{
  auto dp = 0.5 * (p3 - p1);
  auto ddp = p3 - 2.0 * p2 + p1;
  auto denom = std::pow(dp.x() * dp.x() + dp.y() * dp.y(), 3.0 / 2.0);
  if (std::fabs(denom) < 1e-20)
  {
    denom = 1e-20;
  }
  return static_cast<double>((ddp.y() * dp.x() - dp.y() * ddp.x()) / denom);
}

lanelet::BasicPolygon2d objectToMapPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size)
{
  tf2::Transform object_tf;
  tf2::fromMsg(pose, object_tf);

  double half_x_bound = size.x / 2;
  double half_y_bound = size.y / 2;

  // 4 corners of the object starting with upper left and moving in clockwise direction in pose frame
  tf2::Vector3 obj_p1(half_x_bound, half_y_bound, 0);
  tf2::Vector3 obj_p2(half_x_bound, -half_y_bound, 0);
  tf2::Vector3 obj_p3(-half_x_bound, -half_y_bound, 0);
  tf2::Vector3 obj_p4(-half_x_bound, half_y_bound, 0);

  tf2::Vector3 obj_p1_map = object_tf * obj_p1;
  tf2::Vector3 obj_p2_map = object_tf * obj_p2;
  tf2::Vector3 obj_p3_map = object_tf * obj_p3;
  tf2::Vector3 obj_p4_map = object_tf * obj_p4;

  lanelet::BasicPoint2d p1(obj_p1_map.getX(), obj_p1_map.getY());
  lanelet::BasicPoint2d p2(obj_p2_map.getX(), obj_p2_map.getY());
  lanelet::BasicPoint2d p3(obj_p3_map.getX(), obj_p3_map.getY());
  lanelet::BasicPoint2d p4(obj_p4_map.getX(), obj_p4_map.getY());

  return { p1, p2, p3, p4 };
}

lanelet::BasicLineString2d concatenate_lanelets(const std::vector<lanelet::ConstLanelet>& lanelets)
{
  if (lanelets.empty()) {
    return lanelet::BasicLineString2d();
  }
  lanelet::BasicLineString2d centerline_points = lanelets[0].centerline2d().basicLineString();
  lanelet::BasicLineString2d new_points;
  for (size_t i = 1; i < lanelets.size(); i++) {
    new_points = lanelets[i].centerline2d().basicLineString();
    centerline_points = concatenate_line_strings(centerline_points, new_points);
  }

  return centerline_points;
}

template <class P, class A>
std::vector<double>
templated_local_curvatures(const std::vector<P, A>& centerline_points)
{

  if (centerline_points.empty()) {
    throw std::invalid_argument("No points in centerline for curvature calculation");
  }

  std::vector<Eigen::Vector2d> spatial_derivative = compute_finite_differences(centerline_points);
  std::vector<Eigen::Vector2d> normalized_tangent_vectors = normalize_vectors(spatial_derivative);

  std::vector<double> arc_lengths = compute_arc_lengths(centerline_points);

  std::vector<Eigen::Vector2d> tangent_derivative = 
    compute_finite_differences(
      normalized_tangent_vectors, 
      arc_lengths);

  std::vector<double> curvature = compute_magnitude_of_vectors(tangent_derivative);

  return curvature;
}

std::vector<double>
local_curvatures(const lanelet::BasicLineString2d& centerline_points)
{
  return templated_local_curvatures(centerline_points);
}

std::vector<double>
local_curvatures(const std::vector<lanelet::BasicPoint2d>& centerline_points)
{
  return templated_local_curvatures(centerline_points);
}

std::vector<double>
local_curvatures(const std::vector<lanelet::ConstLanelet>& lanelets) {
  return local_curvatures(concatenate_lanelets(lanelets));
}

lanelet::BasicLineString2d 
concatenate_line_strings(const lanelet::BasicLineString2d& a, 
                                          const lanelet::BasicLineString2d& b)
{
  lanelet::BasicLineString2d out;

  int start_offset = 0;
  if (!a.empty() && !b.empty() && compute_euclidean_distance(a.back(), b.front()) < SPATIAL_EPSILON_M) {
    start_offset = 1;
  }

  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin() + start_offset, b.end());

  return out;
}

template <class P, class A>
std::vector<Eigen::Vector2d> 
compute_templated_finite_differences(const std::vector<P,A>& data)
{
  std::vector<Eigen::Vector2d> out;
  Eigen::Vector2d diff;
  Eigen::Vector2d vec_i_plus_1;
  Eigen::Vector2d vec_i_minus_1;

  for (size_t i = 0; i < data.size(); i++) {
    if (i == 0) {
      // Compute forward derivative
      diff = (data[i + 1] - data[i]);
    } else if (i == data.size() - 1) {
      // Compute backward derivative
      diff = (data[i] - data[i - 1]);
    } else {
      // Else, compute centered derivative
      vec_i_plus_1 = data[i + 1];
      vec_i_minus_1 = data[i - 1];
      diff = (vec_i_plus_1 - vec_i_minus_1)/2.0;
    }

    out.push_back(diff);
  }

  return out;
}

std::vector<Eigen::Vector2d> 
compute_finite_differences(const lanelet::BasicLineString2d& data)
{
  return compute_templated_finite_differences(data);
}

std::vector<Eigen::Vector2d> compute_finite_differences(const std::vector<lanelet::BasicPoint2d>& data) {
  return compute_templated_finite_differences(data);
}

std::vector<double> 
compute_finite_differences(const std::vector<double>& data)
{
  std::vector<double> out;
  double diff;
  for (size_t i = 0; i < data.size(); i++) {

    if (i == 0) {
      diff = data[i + 1] - data[i];
    } else if (i == data.size() - 1) {
      diff = data[i] - data[i - 1];
    } else {
      diff = (data[i + 1] - data[i - 1])/2.0;
    }
    
    out.push_back(diff);
  }

  return out;
}

std::vector<Eigen::Vector2d> 
compute_finite_differences(const std::vector<Eigen::Vector2d>& x,const std::vector<double>& y)
{
  if (x.size() != y.size()) {
    throw std::invalid_argument("Attempting to differentiate two unequal sized lists!");
  }

  std::vector<Eigen::Vector2d> out;
  Eigen::Vector2d diff;
  for (size_t i = 0; i < x.size(); i++) {
    if (i == 0) {
      diff = (x[i + 1] - x[i])/(y[i + 1] - y[i]);
    } else if (i == x.size() - 1) {
      diff = (x[i] - x[i - 1])/(y[i] - y[i - 1]);
    } else {
      diff = (x[i + 1] - x[i - 1])/(y[i + 1] - y[i - 1]);
    }
    
    out.push_back(diff);
  }

  return out;
}

template <class P, class A>
std::vector<double>
compute_templated_arc_lengths(const std::vector<P, A>& data) {
  std::vector<double> out;
  double total = 0;
  double diff = 0;
  for (size_t i =  0; i < data.size(); i++) {
    if (i == 0) {
      out.push_back(0);
    } else {
      diff = compute_euclidean_distance(data[i - 1], data[i]);
      total += diff;
      out.push_back(total);
    }
  }

  return out;
}

std::vector<double>
compute_arc_lengths(const lanelet::BasicLineString2d& data) {
  return compute_templated_arc_lengths(data);
}

std::vector<double>
compute_arc_lengths(const std::vector<lanelet::BasicPoint2d>& data)
{
  return compute_templated_arc_lengths(data);
}

double 
compute_euclidean_distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b)
{
  return std::sqrt(std::pow(b[0] - a[0], 2) + std::pow(b[1] - a[1], 2));
}

std::vector<Eigen::Vector2d>
normalize_vectors(const std::vector<Eigen::Vector2d>& vectors)
{
  std::vector<Eigen::Vector2d> out;
  for (auto vec : vectors) {
    out.push_back(vec.normalized());
  }
  return out;
}

std::vector<double> 
compute_magnitude_of_vectors(const std::vector<Eigen::Vector2d>& vectors)
{
  std::vector<double> out;
  for (auto vec : vectors) {
    out.push_back(vec.norm());
  }

  return out;
}

template<class P, class A>
std::vector<double>
compute_templated_tangent_orientations(const std::vector<P,A>& centerline)
{
  std::vector<double> out;
  if (centerline.empty())
    return out;

  std::vector<Eigen::Vector2d> tangents = carma_wm::geometry::compute_templated_finite_differences(centerline);

  for (auto tangent : tangents)
  {
    geometry_msgs::Quaternion q;

    // Derive angle by cos theta = (u . v)/(||u| * ||v||)
    double yaw = 0;
    double norm = tangent.norm();
    if (norm != 0.0) {
      auto normalized_tanged = tangent / norm;
      yaw = atan2(normalized_tanged[1], normalized_tanged[0]);
    }

    out.push_back(yaw);
  }

  return out;
}

std::vector<double>
compute_tangent_orientations(const lanelet::BasicLineString2d& centerline)
{
  return compute_templated_tangent_orientations(centerline);
}

std::vector<double>
compute_tangent_orientations(const std::vector<lanelet::BasicPoint2d>& centerline)
{
  return compute_templated_tangent_orientations(centerline);
}

Eigen::Isometry2d build2dEigenTransform(const Eigen::Vector2d& position, const Eigen::Rotation2Dd& rotation) {
  Eigen::Vector2d scale(1.0, 1.0);
  Eigen::Isometry2d tf;
  return tf.fromPositionOrientationScale(position, rotation, scale);
}

Eigen::Isometry3d build3dEigenTransform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
  Eigen::Vector3d scale(1.0, 1.0, 1.0);
  Eigen::Isometry3d tf;
  return tf.fromPositionOrientationScale(position, rotation, scale);
}

Eigen::Isometry3d build3dEigenTransform(const Eigen::Vector3d& position, const Eigen::AngleAxisd& rotation) {
  Eigen::Vector3d scale(1.0, 1.0, 1.0);
  Eigen::Isometry3d tf;
  return tf.fromPositionOrientationScale(position, rotation, scale);
}

double point_to_point_yaw(const lanelet::BasicPoint2d& cur_point, const lanelet::BasicPoint2d& next_point)
{
  double dx = next_point[0] - cur_point[0];
  double dy = next_point[1] - cur_point[1];
  double yaw = atan2(dy, dx);
  return yaw;
}

double circular_arc_curvature(const lanelet::BasicPoint2d& cur_point, const lanelet::BasicPoint2d& next_point)
{
  double dist = sqrt(pow(cur_point[0] - next_point[0], 2) + pow(cur_point[1] - next_point[1], 2));

  double angle = point_to_point_yaw(cur_point, next_point);

  double r = 0.5 * (dist / std::sin(angle));

  double max_curvature = 100000;
  double curvature = std::min(1 / r, max_curvature);

  return curvature;
}

std::vector<double> local_circular_arc_curvatures(const std::vector<lanelet::BasicPoint2d>& points, int lookahead) {
  std::vector<double> curvatures;
  curvatures.reserve(points.size());

  if (lookahead <= 0) {
    throw std::invalid_argument("local_circular_arc_curvatures lookahead must be greater than 0");
  }

  if (points.empty()) {
    return curvatures;
  }
  else if (points.size() == 1) {
    curvatures.push_back(0.0);
    return curvatures;
  }

  for (size_t i = 0; i < points.size() - 1; i++)
  {
    size_t next_point_index = i + lookahead;
    if (next_point_index >= points.size()) {
      next_point_index = points.size() - 1;
    }
    double cur = circular_arc_curvature(points[i], points[next_point_index]);
    curvatures.push_back(fabs(cur));
  }
  curvatures.push_back(curvatures.back());
  return curvatures;
}


}  // namespace geometry

}  // namespace carma_wm