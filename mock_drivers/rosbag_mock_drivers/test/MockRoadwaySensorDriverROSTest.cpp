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
#include <derived_object_msgs/ObjectWithCovariance.h>
#include <derived_object_msgs/LaneModels.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockRoadwaySensorDriver, roadway_sensor_topic)
{
  ros::NodeHandle nh;

  bool got_obj = false;
  bool got_lane = false;

  ros::Publisher obj_pub = nh.advertise<derived_object_msgs::ObjectWithCovariance>("/bag/hardware_interface/roadway_sensor/detected_objects", 5);

  ros::Subscriber obj_sub =
      nh.subscribe<derived_object_msgs::ObjectWithCovariance>("/hardware_interface/roadway_sensor/detected_objects", 5,
                                     [&](const derived_object_msgs::ObjectWithCovarianceConstPtr& msg) -> void { got_obj = true; });

  ros::Publisher lane_pub = nh.advertise<derived_object_msgs::LaneModels>("/bag/hardware_interface/roadway_sensor/lane_models", 5);

  ros::Subscriber lane_sub =
      nh.subscribe<derived_object_msgs::LaneModels>("/hardware_interface/roadway_sensor/lane_models", 5,
                                     [&](const derived_object_msgs::LaneModelsConstPtr& msg) -> void { got_lane = true; });


  ASSERT_TRUE(testing::waitForSubscribers(obj_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(lane_pub, 2, 10000));

  derived_object_msgs::ObjectWithCovariance msg1;
  derived_object_msgs::LaneModels msg2;
  obj_pub.publish(msg1);
  lane_pub.publish(msg2);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() 
    && !(got_obj && got_lane))
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_obj);
  ASSERT_TRUE(got_lane);
}

}  // namespace mock_drivers

/*!
 * \brief Main entrypoint for unit tests
 */
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mock_roadway_sensor_test");

  auto res = RUN_ALL_TESTS();

  ros::shutdown();

  return res;
}