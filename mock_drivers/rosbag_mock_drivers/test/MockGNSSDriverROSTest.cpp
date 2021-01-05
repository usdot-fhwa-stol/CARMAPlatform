/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <gmock/gmock.h>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockGNSSDriver, gnss_fix_fused_topic)
{
  ros::NodeHandle nh;

  bool got_fix = false;

  ros::Publisher fix_fused_pub =
      nh.advertise<gps_common::GPSFix>("/bag/hardware_interface/gnss_fix_fused", 5);

  ros::Subscriber points_sub = nh.subscribe<gps_common::GPSFix>(
      "/hardware_interface/gnss_fix_fused", 5,
      [&](const gps_common::GPSFixConstPtr& msg) -> void { got_fix = true; });

  ASSERT_TRUE(testing::waitForSubscribers(fix_fused_pub, 2, 10000));

  gps_common::GPSFix msg;
  fix_fused_pub.publish(msg);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() && !got_fix)
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_fix);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_gnss_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}