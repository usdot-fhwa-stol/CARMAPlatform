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

#include "rosbag_mock_drivers/MockDriver.h"

namespace mock_drivers
{
MockDriver::~MockDriver()
{
}

MockDriverNode MockDriver::getMockDriverNode() const
{
  return mock_driver_node_;
}

bool MockDriver::onSpin()
{
  return true;
}

int MockDriver::run() {
  mock_driver_node_.init();

  // driver discovery publisher
  mock_driver_node_.addPub(driver_discovery_pub_ptr_);
  mock_driver_node_.setSpinCallback(std::bind(&MockDriver::spinCallback, this));

  int return_val = onRun();
  
  if (return_val != 0) {
    return return_val;
  }

  mock_driver_node_.spin(getRate());

  return 0;
}

bool MockDriver::spinCallback()
{
  if (last_discovery_pub_ == ros::Time(0) || (ros::Time::now() - last_discovery_pub_).toSec() > 0.95)
  {
    driverDiscovery();
    last_discovery_pub_ = ros::Time::now();
  }
  return onSpin();
}

void MockDriver::driverDiscovery()
{
  cav_msgs::DriverStatus discovery_msg;

  discovery_msg.name = mock_driver_node_.getGraphName();
  discovery_msg.status = getDriverStatus();

  for (DriverType type : getDriverTypes())
  {
    switch (type)
    {
      case DriverType::CAN:
        discovery_msg.can = true;
        break;
      case DriverType::RADAR:
        discovery_msg.radar = true;
        break;
      case DriverType::GNSS:
        discovery_msg.gnss = true;
        break;
      case DriverType::LIDAR:
        discovery_msg.lidar = true;
        break;
      case DriverType::ROADWAY_SENSOR:
        discovery_msg.roadway_sensor = true;
        break;
      case DriverType::COMMS:
        discovery_msg.comms = true;
        break;
      case DriverType::CONTROLLER:
        discovery_msg.controller = true;
        break;
      case DriverType::CAMERA:
        discovery_msg.camera = true;
        break;
      case DriverType::IMU:
        discovery_msg.imu = true;
        break;
      case DriverType::TRAILER_ANGLE_SENSOR:
        discovery_msg.trailer_angle_sensor = true;
        break;
      case DriverType::LIGHTBAR:
        discovery_msg.lightbar = true;
        break;

      default:
        std::invalid_argument("Unsupported DriverType provided by getDriverTypes");
        break;
    }
  }

  mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>(driver_discovery_topic_, discovery_msg);
}
}  // namespace mock_drivers
