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
#include <sensor_msgs/PointCloud2.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockLidarDriver, points_raw_topic)
{
  ros::NodeHandle nh;

  bool got_points = false;

  ros::Publisher bag_points_pub =
      nh.advertise<sensor_msgs::PointCloud2>("/bag/hardware_interface/lidar/points_raw", 5);

  ros::Subscriber points_sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/hardware_interface/lidar/points_raw", 5,
      [&](const sensor_msgs::PointCloud2ConstPtr& msg) -> void { got_points = true; });

  ASSERT_TRUE(testing::waitForSubscribers(bag_points_pub, 2, 10000));

  sensor_msgs::PointCloud2 points;
  bag_points_pub.publish(points);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() && !got_points)
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_points);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_lidar_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}