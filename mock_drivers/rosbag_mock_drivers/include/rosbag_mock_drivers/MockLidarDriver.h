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

#pragma once

#include "rosbag_mock_drivers/MockDriver.h"
#include <sensor_msgs/PointCloud2.h>

namespace mock_drivers
{

/// \brief Mock Lidar driver. Operates as a passthrough for bag data which updates the timestamps on received messages */
class MockLidarDriver : public MockDriver
{
private:
  const std::string points_raw_topic_ = "lidar/points_raw";

protected:
  int onRun() override;

public:
  MockLidarDriver(bool dummy = false);
  ~MockLidarDriver() {};
  std::vector<DriverType> getDriverTypes() override;
  uint8_t getDriverStatus() override;
  unsigned int getRate() override;
};

}  // namespace mock_drivers