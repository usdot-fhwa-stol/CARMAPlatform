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
#include <std_msgs/Float64.h>
#include <j2735_msgs/TransmissionState.h>
#include <geometry_msgs/TwistStamped.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockCANDriver, can_topic)
{
  ros::NodeHandle nh;

  bool got_brake = false;
  bool got_steering_wheel = false;
  bool got_transmission = false;
  bool got_twist = false;

  ros::Publisher bake_pub = nh.advertise<std_msgs::Float64>("/bag/hardware_interface/can/brake_position", 5);

  ros::Subscriber brake_sub =
      nh.subscribe<std_msgs::Float64>("/hardware_interface/can/brake_position", 5,
                                     [&](const std_msgs::Float64ConstPtr& msg) -> void { got_brake = true; });

  ros::Publisher steering_pub = nh.advertise<std_msgs::Float64>("/bag/hardware_interface/can/steering_wheel_angle", 5);

  ros::Subscriber steering_sub =
      nh.subscribe<std_msgs::Float64>("/hardware_interface/can/steering_wheel_angle", 5,
                                     [&](const std_msgs::Float64ConstPtr& msg) -> void { got_steering_wheel = true; });

  ros::Publisher transmission_pub = nh.advertise<j2735_msgs::TransmissionState>("/bag/hardware_interface/can/transmission_state", 5);

  ros::Subscriber transmission_sub =
      nh.subscribe<j2735_msgs::TransmissionState>("/hardware_interface/can/transmission_state", 5,
                                     [&](const j2735_msgs::TransmissionStateConstPtr& msg) -> void { got_transmission = true; });

  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/bag/hardware_interface/vehicle/twist", 5);

  ros::Subscriber twist_sub =
      nh.subscribe<geometry_msgs::TwistStamped>("/hardware_interface/vehicle/twist", 5,
                                     [&](const geometry_msgs::TwistStampedConstPtr& msg) -> void { got_twist = true; });

  ASSERT_TRUE(testing::waitForSubscribers(bake_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(steering_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(transmission_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(twist_pub, 2, 10000));

  std_msgs::Float64 msg1;
  std_msgs::Float64 msg2;
  j2735_msgs::TransmissionState msg3;
  geometry_msgs::TwistStamped msg4;

  bake_pub.publish(msg1);
  steering_pub.publish(msg2);
  transmission_pub.publish(msg3);
  twist_pub.publish(msg4);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() 
    && !(got_brake && got_steering_wheel && got_transmission && got_twist))
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_brake);
  ASSERT_TRUE(got_steering_wheel);
  ASSERT_TRUE(got_transmission);
  ASSERT_TRUE(got_twist);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_can_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}