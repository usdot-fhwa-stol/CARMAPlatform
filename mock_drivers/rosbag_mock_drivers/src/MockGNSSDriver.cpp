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

#include "rosbag_mock_drivers/MockGNSSDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockGNSSDriver::getDriverTypes()
{
  return { DriverType::GNSS };
}

uint8_t MockGNSSDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockGNSSDriver::MockGNSSDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockGNSSDriver::getRate()
{
  return 20;  // 20 Hz as default spin rate to match expected gnss data rate
}

int MockGNSSDriver::onRun()
{
  // driver publisher and subscriber
  addPassthroughPub<gps_common::GPSFix>(bag_prefix_ + gnss_fix_fuxed_topic_, gnss_fix_fuxed_topic_, false, 10);

  return 0;
}

}  // namespace mock_drivers