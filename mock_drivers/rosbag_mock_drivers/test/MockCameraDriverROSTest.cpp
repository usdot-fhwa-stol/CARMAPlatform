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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/ProjectionMatrix.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockCameraDriver, camera_topic)
{
  ros::NodeHandle nh;

  bool got_info = false;
  bool got_raw = false;
  bool got_rect = false;
  bool got_mat = false;

  ros::Publisher info_pub = nh.advertise<sensor_msgs::CameraInfo>("/bag/hardware_interface/camera/camera_info", 5);

  ros::Subscriber info_sub =
      nh.subscribe<sensor_msgs::CameraInfo>("/hardware_interface/camera/camera_info", 5,
                                     [&](const sensor_msgs::CameraInfoConstPtr& msg) -> void { got_info = true; });

  ros::Publisher image_raw_pub = nh.advertise<sensor_msgs::Image>("/bag/hardware_interface/camera/image_raw", 5);

  ros::Subscriber image_raw_sub =
      nh.subscribe<sensor_msgs::Image>("/hardware_interface/camera/image_raw", 5,
                                     [&](const sensor_msgs::ImageConstPtr& msg) -> void { got_raw = true; });

  ros::Publisher rect_pub = nh.advertise<sensor_msgs::Image>("/bag/hardware_interface/camera/image_rect", 5);

  ros::Subscriber rect_sub =
      nh.subscribe<sensor_msgs::Image>("/hardware_interface/camera/image_rect", 5,
                                     [&](const sensor_msgs::ImageConstPtr& msg) -> void { got_rect = true; });

  ros::Publisher proj_pub = nh.advertise<autoware_msgs::ProjectionMatrix>("/bag/hardware_interface/camera/projection_matrix", 5);

  ros::Subscriber proj_sub =
      nh.subscribe<autoware_msgs::ProjectionMatrix>("/hardware_interface/camera/projection_matrix", 5,
                                     [&](const autoware_msgs::ProjectionMatrixConstPtr& msg) -> void { got_mat = true; });

  ASSERT_TRUE(testing::waitForSubscribers(info_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(image_raw_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(rect_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(proj_pub, 2, 10000));

  sensor_msgs::CameraInfo msg1;
  sensor_msgs::Image msg2;
  sensor_msgs::Image msg3;
  autoware_msgs::ProjectionMatrix msg4;
  info_pub.publish(msg1);
  image_raw_pub.publish(msg2);
  rect_pub.publish(msg3);
  proj_pub.publish(msg4);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() 
    && !(got_info && got_raw && got_rect && got_mat))
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_info);
  ASSERT_TRUE(got_raw);
  ASSERT_TRUE(got_rect);
  ASSERT_TRUE(got_mat);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_camera_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}