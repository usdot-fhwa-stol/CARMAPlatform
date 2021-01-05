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
#include <radar_msgs/RadarStatus.h>
#include <radar_msgs/RadarTrackArray.h>
#include "test_utils.h"

namespace mock_drivers
{
TEST(MockRadarDriver, radar_topic)
{
  ros::NodeHandle nh;

  bool got_status = false;
  bool got_tracks = false;

  ros::Publisher status_pub = nh.advertise<radar_msgs::RadarStatus>("/bag/hardware_interface/radar/status", 5);

  ros::Subscriber status_sub =
      nh.subscribe<radar_msgs::RadarStatus>("/hardware_interface/radar/status", 5,
                                     [&](const radar_msgs::RadarStatusConstPtr& msg) -> void { got_status = true; });

  ros::Publisher tracks_pub = nh.advertise<radar_msgs::RadarTrackArray>("/bag/hardware_interface/radar/tracks_raw", 5);

  ros::Subscriber tracks_sub =
      nh.subscribe<radar_msgs::RadarTrackArray>("/hardware_interface/radar/tracks_raw", 5,
                                     [&](const radar_msgs::RadarTrackArrayConstPtr& msg) -> void { got_tracks = true; });


  ASSERT_TRUE(testing::waitForSubscribers(status_pub, 2, 10000));
  ASSERT_TRUE(testing::waitForSubscribers(tracks_pub, 2, 10000));

  radar_msgs::RadarStatus msg1;
  radar_msgs::RadarTrackArray msg2;
  status_pub.publish(msg1);
  tracks_pub.publish(msg2);

  ros::Rate r(10);  // 10 hz
  ros::WallTime endTime = ros::WallTime::now() + ros::WallDuration(10.0);
  while (ros::ok() && endTime > ros::WallTime::now() 
    && !(got_status && got_tracks))
  {
    ros::spinOnce();
    r.sleep();
  }

  ASSERT_TRUE(got_status);
  ASSERT_TRUE(got_tracks);
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