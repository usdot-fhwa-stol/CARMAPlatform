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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <j2735_msgs/TransmissionState.h>
#include <cav_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <automotive_platform_msgs/VelocityAccel.h>

namespace mock_drivers
{
/*! \brief Mock CAN driver. Operates as a passthrough for bag data which updates the timestamps on received messages */
class MockCANDriver : public MockDriver
{
private:
  const std::string brake_position_topic_ = "can/brake_position";
  const std::string steering_wheel_angle_topic_ = "can/steering_wheel_angle";
  const std::string transmission_state_topic_ = "can/transmission_state";
  const std::string vehicle_twist = "vehicle/twist";

protected:
  int onRun() override;

public:
  MockCANDriver(bool dummy = false);
  ~MockCANDriver() {};
  std::vector<DriverType> getDriverTypes() override;
  uint8_t getDriverStatus() override;
  unsigned int getRate() override;
};

}  // namespace mock_drivers