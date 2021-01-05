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

#include "rosbag_mock_drivers/MockIMUDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockIMUDriver::getDriverTypes()
{
  return { DriverType::IMU };
}

uint8_t MockIMUDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockIMUDriver::MockIMUDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockIMUDriver::getRate()
{
  return 100;  // 100 Hz as default spin rate to match expected imu data rate
}

int MockIMUDriver::onRun()
{
  // main driver publisher
  addPassthroughPub<sensor_msgs::Imu>(bag_prefix_ + raw_data_topic_, raw_data_topic_, false, 10);

  return 0;
}

}  // namespace mock_drivers