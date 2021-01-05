#pragma once

/*
 * Copyright (C) 2018-2021 LEIDOS.
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
#include <tf2/LinearMath/Transform.h>
#include <wgs84_utils/wgs84_utils.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps_msgs/NovatelDualAntennaHeading.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <gps_common/GPSFix.h>

/**
 * \class GNSSToMapNode
 * \brief ROS Node which maintains a tf2 transform tree which can be accessed by other nodes.
 *
 * The get_transform service can be used to obtain coordinate transformations between two frames.
 * Only transforms published on the /tf or /tf_static topics are recorded by this node.
 */
namespace gnss_to_map_convertor
{
  /**
   * \brief Generates a pose for a vehicle in a map from given the transform to that frame and the gnss fix and heading
   * 
   * \param baselink_in_sensor Transform describing the location of the base_link frame in the gnss sensor frame
   * \param sensor_in_ned_heading Transform describing the location of the sensor frame in the frame in which that sensor reports its heading
   * \param fix_msg The message containing gnss fix and heading information
   * 
   * \return The pose in the map frame
   */ 
  geometry_msgs::PoseWithCovarianceStamped poseFromGnss(
    const tf2::Transform& baselink_in_sensor,
    const tf2::Transform& sensor_in_ned_heading,
    const gps_common::GPSFixConstPtr& fix_msg
  );

  /**
   * \brief Generates a pose message for a vehicle in the map frame when provided with that vehicles location in the earth frame and the maps location in the earth frame
   * 
   * \param baselink_in_earth Transform describing location of base_link frame in the earth frame
   * \param map_in_earth Transform describing location of map frame in the earth frame
   * 
   * \return geometry_msgs::Pose containing the pose of the vehicle in the map frame
   */ 
  geometry_msgs::Pose ecefTFToMapPose(const tf2::Transform& baselink_in_earth, const tf2::Transform& map_in_earth);

};