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

#include <gtest/gtest.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "gnss_to_map_convertor/GNSSToMapConvertor.h"

void assertNear(const tf2::Transform& tf, const geometry_msgs::Pose& pose, double error_bound_dist, double error_bound_rad) {
  ASSERT_NEAR(tf.getOrigin().getX(), pose.position.x, error_bound_dist);
  ASSERT_NEAR(tf.getOrigin().getY(), pose.position.y, error_bound_dist);
  ASSERT_NEAR(tf.getOrigin().getZ(), pose.position.z, error_bound_dist);

  ASSERT_NEAR(tf.getRotation().getX(), pose.orientation.x, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getY(), pose.orientation.y, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getZ(), pose.orientation.z, error_bound_rad);
  ASSERT_NEAR(tf.getRotation().getW(), pose.orientation.w, error_bound_rad);
}

/**
 * Tests the ecefTFToMapPose function of the gnss_to_map_convertor namespace
 */ 
TEST(GNSSToMapConvertor, ecefTFToMapPose)
{
  tf2::Vector3 transT_e_b(2,3,0);
  tf2::Transform baselink_in_earth(tf2::Quaternion::getIdentity(), transT_e_b);

  tf2::Vector3 transT_e_m(3,1,0);
  tf2::Quaternion quatT_e_m(0, 0, 0.7071068, 0.7071068);
  tf2::Transform map_in_earth(quatT_e_m, transT_e_m);

  tf2::Vector3 transT_m_b(2,1,0);
  tf2::Quaternion quatT_m_b(0, 0, -0.7071068, 0.7071068);
  tf2::Transform base_link_in_map(quatT_m_b, transT_m_b);

  geometry_msgs::Pose pose = gnss_to_map_convertor::ecefTFToMapPose(baselink_in_earth, map_in_earth);

  double error_bound_dist = 0.0000001;
  double error_bound_rad = 0.0000001;
  assertNear(base_link_in_map, pose, error_bound_dist, error_bound_rad);

}

TEST(GNSSToMapConvertor, poseFromGnss)
{
  tf2::Transform baselink_in_sensor = tf2::Transform::getIdentity();
  tf2::Transform sensor_in_ned_heading = tf2::Transform::getIdentity();
  tf2::Quaternion sensor_in_ned_heading_quat;
  sensor_in_ned_heading_quat.setRPY(wgs84_utils::pi, 0, 0); 
  sensor_in_ned_heading.setRotation(sensor_in_ned_heading_quat);

  gps_common::GPSFix fix_msg;

  // Test point at prime meridian and equator with 0 heading
  fix_msg.latitude = 0;
  fix_msg.longitude = 0;
  fix_msg.altitude = 0;

  fix_msg.track = 0;

  gps_common::GPSFixConstPtr fix_ptr(new gps_common::GPSFix(fix_msg));

  geometry_msgs::PoseWithCovarianceStamped result;
  result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr);

  tf2::Vector3 solTrans(6378137.0, 0, 0);
  tf2::Quaternion solRot;
  solRot.setRPY(180.0 * wgs84_utils::DEG2RAD, -90.0 * wgs84_utils::DEG2RAD, 0); // setRPY uses extrinsic fixed frame not intrinsic frame
  tf2::Transform solution(solRot, solTrans);
  
  double error_bound_dist = 1;
  double error_bound_rad = 0.000001;
  assertNear(solution, result.pose.pose, error_bound_dist, error_bound_rad);

  // Test point at prime meridian and equator with offset heading
  fix_msg.latitude = 0;
  fix_msg.longitude = 0;
  fix_msg.altitude = 0;

  fix_msg.track = 90.0;

  fix_ptr = gps_common::GPSFixConstPtr(new gps_common::GPSFix(fix_msg));

  result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr);

  tf2::Vector3 solTrans2(6378137.0, 0, 0);
  tf2::Quaternion solRot2;
  solRot2.setRPY(90.0 * wgs84_utils::DEG2RAD, 0, 90.0 * wgs84_utils::DEG2RAD);
  tf2::Transform solution2(solRot2, solTrans2);

  assertNear(solution2, result.pose.pose, error_bound_dist, error_bound_rad);
  
  //////// TODO pitch is not currently accounted for in the function under test. This section can be used when it is.
  // Test point at prime meridian and equator with offset pitch
  // fix_msg.latitude = 0;
  // fix_msg.longitude = 0;
  // fix_msg.altitude = 0;

  // heading_msg.heading = 0;
  // heading_msg.pitch = 90.0 * wgs84_utils::DEG2RAD;

  // fix_ptr = sensor_msgs::NavSatFixConstPtr(new sensor_msgs::NavSatFix(fix_msg));
  // heading_ptr = novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr(new novatel_gps_msgs::NovatelDualAntennaHeading(heading_msg));

  // result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr, heading_ptr);

  // tf2::Vector3 solTrans3(6378137.0, 0, 0);
  // tf2::Quaternion solRot3;
  // solRot3.setRPY(180.0 * wgs84_utils::DEG2RAD, 0, 0);
  // tf2::Transform solution3(solRot3, solTrans3);

  //assertNear(solution3, result.pose.pose, error_bound_dist, error_bound_rad);


  // Test point at prime meridian and equator with offset pitch and heading
  // fix_msg.latitude = 0;
  // fix_msg.longitude = 0;
  // fix_msg.altitude = 0;

  // heading_msg.heading = 90.0 * wgs84_utils::DEG2RAD;
  // heading_msg.pitch = 90.0 * wgs84_utils::DEG2RAD;

  // fix_ptr = sensor_msgs::NavSatFixConstPtr(new sensor_msgs::NavSatFix(fix_msg));
  // heading_ptr = novatel_gps_msgs::NovatelDualAntennaHeadingConstPtr(new novatel_gps_msgs::NovatelDualAntennaHeading(heading_msg));

  // result = gnss_to_map_convertor::poseFromGnss(baselink_in_sensor, sensor_in_ned_heading, fix_ptr, heading_ptr);

  // tf2::Vector3 solTrans4(6378137.0, 0, 0);
  // tf2::Quaternion solRot4;
  // solRot4.setRPY(90.0 * wgs84_utils::DEG2RAD, 0, 0);
  // tf2::Transform solution4(solRot4, solTrans4);

  //assertNear(solution4, result.pose.pose, error_bound_dist, error_bound_rad);
  /////////
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
