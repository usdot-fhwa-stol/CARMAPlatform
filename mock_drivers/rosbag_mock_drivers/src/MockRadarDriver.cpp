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

#include "rosbag_mock_drivers/MockRadarDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockRadarDriver::getDriverTypes()
{
  return { DriverType::RADAR };
}

uint8_t MockRadarDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockRadarDriver::MockRadarDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockRadarDriver::getRate()
{
  return 20;  // 20 Hz as default spin rate to match expected radar data rate
}

int MockRadarDriver::onRun()
{

  // driver publisher and subscriber
  addPassthroughPub<radar_msgs::RadarStatus>(bag_prefix_ + radar_status_topic_, radar_status_topic_, false, 10);
  addPassthroughPub<radar_msgs::RadarTrackArray>(bag_prefix_ + radar_tracks_raw_topic_, radar_tracks_raw_topic_, false,
                                                 10);

  return 0;
}

}  // namespace mock_drivers