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
#include <tuple>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_core/utility/Optional.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "TrackPos.h"

namespace carma_wm
{
/**
 * \brief geometry namespace contains utility geometry functions which do not require the map or route provided by an
 * active world model
 */
namespace geometry
{
/*! \brief Returns the TrackPos, computed in 2d, of the provided point relative to the centerline of the provided
 * lanelet. Positive crosstrack will be to the right. Points occuring before the segment will have negative downtrack.
 *        See the matchSegment function for a description of the edge cases associated with the max_crosstrack
 * parameter
 *
 * \param lanelet The lanelet whose centerline will serve as the TrackPos reference line
 * \param point The lanelet2 point which will have its distance computed
 * \param max_crosstrack The maximum crosstrack distance allowed for a perfect centerline segment match. See the
 * matchSegment function for details
 *
 * \throws std::invalid_argument If the lanelet centerline cannot be found
 *
 * \return The TrackPos of the point relative to the lanelet centerline
 */
TrackPos trackPos(const lanelet::ConstLanelet& lanelet, const lanelet::BasicPoint2d& point);

/*! \brief Return the track position of a point relative to a line segment defined by two other points.
 *         Positive crosstrack will be to the right. Points occuring before the segment will have negative downtrack
 *
 *  \param p The point to find the TrackPos of
 *  \param seg_start The starting point of a line segment
 *  \param seg_end The ending point of a line segment
 *
 *  \return The TrackPos of point p relative to the line defined by seg_start and seg_end
 */
TrackPos trackPos(const lanelet::BasicPoint2d& p, const lanelet::BasicPoint2d& seg_start,
                  const lanelet::BasicPoint2d& seg_end);
/**
 * \brief Get the TrackPos of the provided point along the linestring and the segment of the line string which the
 * provided point should be considered in.
 *
 * This function iterates over the provided linestring from front to back.
 * First the nearest vertex point on the linestring to the external point is found
 * Once the nearest point has been found next step is to determine which segment it should go with using the following
 * rules. If the minimum point is the first point then use the first segment If the minimum point is the last point
 * then use the last segment If the minimum point is within the downtrack bounds of one segment but not the other then
 * use the one it is within If the minimum point is within the downtrack bounds of both segments then use the one with
 * the smallest crosstrack distance If the minimum point is within the downtrack bounds of both segments and has
 * exactly equal crosstrack bounds with each segment then use the first one
 *
 * \param point The 2d point to match with a segment
 * \param line_string The line_string to match against
 *
 * \throw std::invalid_argument if line string contains only one point
 *
 * \return An std::tuple where the first element is the TrackPos of the point and the second element is the matched
 * segment
 */
std::tuple<TrackPos, lanelet::BasicSegment2d> matchSegment(const lanelet::BasicPoint2d& p,
                                                           const lanelet::BasicLineString2d& line_string);

/*! \brief Returns a list of local (computed by discrete derivative)
 * curvatures for the input centerlines. The list of returned curvatures matches
 * 1-to-1 with with list of points in the input centerlines.
 *
 * The numerical accuracy of this method is greatly increased by using larger
 * collections of points as inputs as the first 2 and final 2 points in the
 * list must be computed using alternative differentiation methods from all
 * the others resulting in greater error. It is also important for this function
 * to yield useful results that all points in the input lanelet's centerline
 * are actually on the centerline of the lanelet (i.e. none to minimal linear
 * interpolation of points) as this results in "flat" spots on an otherwise
 * smooth curve that causes 0 curvature to be computed.
 *
 * NOTE: The accuracy of this method is very susceptable to input noise.
 *       Consider using local_circular_arc_curvatures for calculations using noisy data
 *
 * \param centerline_points The list of points to compute curvatures for
 *
 * \throw std::invalid_argument If one of the provided lanelets cannot have its centerline computed
 *
 * \return A list of continuous centerline segments and their respective curvatures
 */
std::vector<double> local_curvatures(const lanelet::BasicLineString2d& centerline_points);

std::vector<double> local_curvatures(const std::vector<lanelet::BasicPoint2d>& centerline_points);

/*!
 * \brief Specialized overload of the centerline local_curvatures method but for lanelets
 *
 * \param lanelets The list of points to compute curvatures for
 *
 * \throw std::invalid_argument If one of the provided lanelets cannot have its centerline computed
 */
std::vector<double> local_curvatures(const std::vector<lanelet::ConstLanelet>& lanelets);

/*!
 * \brief Helper function to concatenate 2 linestrings together and return the result. Neither LineString is modified in
 * this function.
 */
lanelet::BasicLineString2d concatenate_line_strings(const lanelet::BasicLineString2d& l1,
                                                    const lanelet::BasicLineString2d& l2);

/*!
 * \brief Helper function to a list of lanelets together and return the result. Neither LineString is modified in this
 * function.
 */
lanelet::BasicLineString2d concatenate_lanelets(const std::vector<lanelet::ConstLanelet>& lanelets);

/*!
 * \brief Use finite differences methods to compute the derivative of the input data set with respect to index
 * Compute the finite differences using the forward finite difference for the first point, centered finite differences
 * for the middle points and backwards finite difference for the final point. This will result in the middle points
 * being a better approximation of the actual derivative than the endpoints.
 *
 * \param data The data to differentiate over
 * \return A vector containing the point-by-point derivatives in the same indices as the input data
 */
std::vector<Eigen::Vector2d> compute_finite_differences(const lanelet::BasicLineString2d& data);

std::vector<Eigen::Vector2d> compute_finite_differences(const std::vector<lanelet::BasicPoint2d>& data);

/*!
 * \brief Use finite differences methods to compute the derivative of the input data set with respect to index
 * Compute the finite differences using the forward finite difference for the first point, centered finite differences
 * for the middle points and backwards finite difference for the final point. This will result in the middle points
 * being a better approximation of the actual derivative than the endpoints.
 *
 * \param data The data to differentiate over
 * \return A vector containing the point-by-point derivatives in the same indices as the input data
 */
std::vector<double> compute_finite_differences(const std::vector<double>& data);

/*!
 * \brief Use finite differences methods to compute the derivative of the input data set with respect to the second
 * paramter.
 *
 * Input x and y must be the same length. Compute the finite differences using the forward finite difference for the
 * first point, centered finite differences for the middle points and backwards finite difference for the final point.
 * This will result in the middle points being a better approximation of the actual derivative than the endpoints.
 *
 * \param x The x value of the derivative dx/dy
 * \param y The y value of the derivative dx/dy
 * \return A vector containing the point-by-point derivatives in the same indices as the input data
 */
std::vector<Eigen::Vector2d> compute_finite_differences(const std::vector<Eigen::Vector2d>& x,
                                                        const std::vector<double>& y);

/*!
 * \brief Compute the arc length at each point around the curve
 */
std::vector<double> compute_arc_lengths(const std::vector<lanelet::BasicPoint2d>& data);

/*!
 * \brief Compute the arc length at each point around the curve
 */
std::vector<double> compute_arc_lengths(const lanelet::BasicLineString2d& data);

/*!
 * \brief Compute the Euclidean distance between the two points
 */
double compute_euclidean_distance(const Eigen::Vector2d& a, const Eigen::Vector2d& b);

/*!
 * \brief Normalize the vectors in the input list such that their magnitudes = 1
 */
std::vector<Eigen::Vector2d> normalize_vectors(const std::vector<Eigen::Vector2d>& vectors);

/*!
 * \brief Compute the magnitude of each vector in the input list
 */
std::vector<double> compute_magnitude_of_vectors(const std::vector<Eigen::Vector2d>& vectors);

/*! \brief Function for computing curvature from 3 points.
 *
 * This function is a direct copy of the function by the same name found in the lanelet2_validation package which was
 * not exposed for use The original function can be found here
 * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_validation/src/validators/CurvatureTooBig.cpp
 * The source function is copyrighted under BSD 3-Clause "New" or "Revised" License a copy of that notice has been
 * included with this package
 *
 * \param p1 The first point
 * \param p2 The second point
 * \param p3 The third point
 *
 * \return The computed curvature in 1/m units.
 */
[[deprecated("computeCurvature is deprecated in favor of using the finite differences-based computeLocalCurvature for "
             "large curves")]] double
computeCurvature(const lanelet::BasicPoint2d& p1, const lanelet::BasicPoint2d& p2, const lanelet::BasicPoint2d& p3);

/**
 * \brief acos method that caps the input x value for 1 or -1 to avoid undefined output.
 *        This is meant to prevent issues with floating point approximations.
 *        The user should still make sure the input data is expected to be within the [-1, 1] range for this method to
 * give accurate results.
 *
 * \param x The angle in radians
 *
 * \return The arc cosine of x
 */
double safeAcos(double x);

/**
 * \brief Calculates the angle between two vectors.
 *
 * \param vec1 the first vector
 * \param vec2 the second vector
 *
 * \return The angle in rad between the two vectors
 */
double getAngleBetweenVectors(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2);

/**
 * \brief Uses the provided pose and size vector of an ExternalObject to compute what the polygon would be of that
 * object would be if viewed from the same frame as the pose is defined relative to.
 *
 * \param pose the pose of an external object
 * \param size The size vector of an external object
 *
 * \return A polygon of 4 points describing the object aligned bounds starting with the upper left point in the object
 * frame and moving clockwise.
 */
lanelet::BasicPolygon2d objectToMapPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size);

/**
 * \brief Extract extrinsic roll-pitch-yaw from quaternion
 *
 * \param q The quaternion to convert
 * \param roll The output variable for roll units will be radians
 * \param pitch The output variable for pitch units will be radians
 * \param yaw The output variable for yaw units will be radians
 *
 */
void rpyFromQuaternion(const tf2::Quaternion& q, double& roll, double& pitch, double& yaw);

/**
 * \brief Extract extrinsic roll-pitch-yaw from quaternion
 *
 * \param q The quaternion message to convert
 * \param roll The output variable for roll units will be radians
 * \param pitch The output variable for pitch units will be radians
 * \param yaw The output variable for yaw units will be radians
 *
 */
void rpyFromQuaternion(const geometry_msgs::Quaternion& q_msg, double& roll, double& pitch, double& yaw);

/**!
 * \brief Compute an approximate orientation for the vehicle at each
 * point along the provided centerline.
 *
 * Uses the tangent vectors along the centerline to estimate vehicle pose
 * in the yaw dimension. Roll and pitch are not considered.
 *
 * \param centerline centerline represented as BasicLinestring2d to compute the orientation
 *
 * \returns A vector of doubles representing the yaw value in radians the same frame as the provided centerline.
 *
 */
std::vector<double> compute_tangent_orientations(const lanelet::BasicLineString2d& centerline);

std::vector<double> compute_tangent_orientations(const std::vector<lanelet::BasicPoint2d>& centerline);

/**
 * \brief Builds a 2D Eigen coordinate frame transform with not applied scaling (only translation and rotation)
 *        based on the provided position and rotation parameters
 *
 * \param position The origin of the child coordinate frame in the parent frame
 * \param rotation The rotation of the child coordinate frame in the parent frame
 *
 * \return A 2D transformation which is defined such that given a parent frame F and a child frame C,
 *         A point P_c in child frame C can be converted to the parent frame using the returned transform T_f_c
 *         Therefore P_f = T_f_c * P_c
 */
Eigen::Isometry2d build2dEigenTransform(const Eigen::Vector2d& position, const Eigen::Rotation2Dd& rotation);

/**
 * \brief Builds a 3D Eigen coordinate frame transform with not applied scaling (only translation and rotation)
 *        based on the provided position and rotation parameters
 *
 * \param position The origin of the child coordinate frame in the parent frame
 * \param rotation The rotation of the child coordinate frame in the parent frame
 *
 * \return A 3D transformation which is defined such that given a parent frame F and a child frame C,
 *         A point P_c in child frame C can be converted to the parent frame using the returned transform T_f_c
 *         Therefore P_f = T_f_c * P_c
 */
Eigen::Isometry3d build3dEigenTransform(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation);

Eigen::Isometry3d build3dEigenTransform(const Eigen::Vector3d& position, const Eigen::AngleAxisd& rotation);

/**
 * \brief Computes the 2d orientation of the vector from the provided start point to end point
 *
 * \param cur_point The start point of the orientation vector. Units m
 * \param next_point The end point of the orientation vector. Units m
 *
 * \return The rotation of the vector around the +Z axis in radians
 */
double point_to_point_yaw(const lanelet::BasicPoint2d& cur_point, const lanelet::BasicPoint2d& next_point);

/**
 * \brief Computes the curvature of a circular arc that passes through the two provided points.
 *        If the two points lie on a curve then larger distances between them will result in less accurate, but also
 * less noisy curvature calculations.
 *
 * \param cur_point The first point
 * \param next_point The second point
 *
 * \return The curvature of the circular arc that passes through both points. Curvature is always positive.
 */
double circular_arc_curvature(const lanelet::BasicPoint2d& cur_point, const lanelet::BasicPoint2d& next_point);

/**
 * \brief Computes the curvature of each point in the provided list by calling circular_arc_curvature
 *        using that point and the point at index current point index + lookahead.
 *        In this way the curvature of a path is effectively low pass filtered by the size of the lookahead value
 *
 * \param points The points to compute the curvature for
 * \param lookahead The lookahead index distance to use for computing the curvature at each point
 *
 * \return A vector of computed curvatures that has size equal to points.size()
 *
 */
std::vector<double> local_circular_arc_curvatures(const std::vector<lanelet::BasicPoint2d>& points, int lookahead);

}  // namespace geometry

}  // namespace carma_wm