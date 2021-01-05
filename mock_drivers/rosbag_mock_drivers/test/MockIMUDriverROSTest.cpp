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
#include <sensor_msgs/Imu.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockImuDriver, imu_topic)
{
  ros::NodeHandle nh;

  bool got_imu = false;

  ros::Publisher pub =
      nh.advertise<sensor_msgs::Imu>("/bag/hardware_interface/imu", 5);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Imu>(
      "/hardware_interface/imu", 5,
      [&](const sensor_msgs::ImuConstPtr& msg) -> void { got_imu = true; });

  ASSERT_TRUE(testing::waitForSubscribers(pub, 2, 10000));

  sensor_msgs::Imu msg;
  pub.publish(msg);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() && !got_imu)
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_imu);
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