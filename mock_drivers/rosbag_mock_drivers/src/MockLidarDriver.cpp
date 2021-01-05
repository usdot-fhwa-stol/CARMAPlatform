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

#include "rosbag_mock_drivers/MockLidarDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockLidarDriver::getDriverTypes()
{
  return { DriverType::LIDAR };
}

uint8_t MockLidarDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockLidarDriver::MockLidarDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockLidarDriver::getRate()
{
  return 20;  // 20 Hz as default spin rate to surpass expected lidar data rate
}

int MockLidarDriver::onRun()
{
  // driver publisher and subscriber
  addPassthroughPub<sensor_msgs::PointCloud2>(bag_prefix_ + points_raw_topic_, points_raw_topic_, false, 10);

  return 0;
}

}  // namespace mock_drivers