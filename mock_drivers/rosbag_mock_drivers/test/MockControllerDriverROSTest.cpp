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
#include <autoware_msgs/VehicleCmd.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_srvs/SetEnableRobotic.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockControllerDriver, controller_topic)
{
  ros::NodeHandle nh;

  bool enabled = false;
  bool active = false;

  ros::Publisher cmd_pub =
      nh.advertise<autoware_msgs::VehicleCmd>("/hardware_interface/controller/vehicle_cmd", 5);

  ros::Subscriber robot_enabled_sub = nh.subscribe<cav_msgs::RobotEnabled>(
      "/hardware_interface/controller/robot_status", 5, [&](const cav_msgs::RobotEnabledConstPtr& msg) -> void {
        enabled = msg->robot_enabled;
        active = msg->robot_active;
      });

  ros::ServiceClient client = nh.serviceClient<cav_srvs::SetEnableRobotic>("/hardware_interface/controller/enable_robotic");

  ASSERT_TRUE(testing::waitForSubscribers(cmd_pub, 1, 10000));
  ASSERT_TRUE(ros::service::waitForService("/hardware_interface/controller/enable_robotic", ros::Duration(10.0)));

  autoware_msgs::VehicleCmd msg1;
  cmd_pub.publish(msg1);

  cav_srvs::SetEnableRobotic srv;
  srv.request.set = cav_srvs::SetEnableRobotic::Request::ENABLE;
  ASSERT_TRUE(client.call(srv));

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() && !(enabled && active))
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(enabled);
  ASSERT_TRUE(active);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_radar_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}