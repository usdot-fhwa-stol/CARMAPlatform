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

#include <gnss_to_map_convertor/GNSSToMapConvertor.h>

namespace gnss_to_map_convertor {

  geometry_msgs::Pose ecefTFToMapPose(const tf2::Transform& baselink_in_earth, const tf2::Transform& map_in_earth){

    const tf2::Transform T_m_e = map_in_earth.inverse(); // T_e_m^-1 = T_m_e

    const tf2::Transform T_m_b = T_m_e * baselink_in_earth; // T_m_b = T_m_e * T_e_b
  
    geometry_msgs::Pose pose;
    pose.position.x = T_m_b.getOrigin().getX();
    pose.position.y = T_m_b.getOrigin().getY();
    pose.position.z = T_m_b.getOrigin().getZ();

    pose.orientation.x = T_m_b.getRotation().getX();
    pose.orientation.y = T_m_b.getRotation().getY();
    pose.orientation.z = T_m_b.getRotation().getZ();
    pose.orientation.w = T_m_b.getRotation().getW();

    return pose;
  }

  geometry_msgs::PoseWithCovarianceStamped poseFromGnss(
    const tf2::Transform& baselink_in_sensor,
    const tf2::Transform& sensor_in_ned_heading,
    const gps_common::GPSFixConstPtr& fix_msg
  ) {
    const double lat = fix_msg->latitude * wgs84_utils::DEG2RAD;
    const double lon = fix_msg->longitude * wgs84_utils::DEG2RAD;
    const double alt = fix_msg->altitude;

    const struct wgs84_utils::wgs84_coordinate geo_point = {lat, lon, 0, alt};

    const tf2::Transform T_e_n = wgs84_utils::ecef_to_ned_from_loc(geo_point);

    const tf2::Vector3 identity_trans(0,0,0);
    tf2::Quaternion heading_in_ned_quat;
    
    heading_in_ned_quat.setRPY(0, 0, fix_msg->track * wgs84_utils::DEG2RAD);

    const tf2::Transform T_n_h(heading_in_ned_quat, identity_trans);

    const tf2::Transform T_h_s(sensor_in_ned_heading);

    const tf2::Transform T_n_s = T_n_h * T_h_s; // Transform defining sensor orientation in NED frame

    const tf2::Transform T_s_b(baselink_in_sensor);

     const tf2::Transform T_n_b = T_n_s * T_s_b; // Transform defining vehicle orientation in NED frame

    // TODO handle covariance

    const tf2::Transform T_e_b = T_e_n * T_n_b;

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header = fix_msg->header;

    pose.pose.pose.position.x = T_e_b.getOrigin().getX();
    pose.pose.pose.position.y = T_e_b.getOrigin().getY();
    pose.pose.pose.position.z = T_e_b.getOrigin().getZ();

    pose.pose.pose.orientation.x = T_e_b.getRotation().getX();
    pose.pose.pose.orientation.y = T_e_b.getRotation().getY();
    pose.pose.pose.orientation.z = T_e_b.getRotation().getZ();
    pose.pose.pose.orientation.w = T_e_b.getRotation().getW();

    return pose;
  }
}
