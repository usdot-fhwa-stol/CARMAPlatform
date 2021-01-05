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

#include "rosbag_mock_drivers/MockCameraDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockCameraDriver::getDriverTypes()
{
  return { DriverType::CAMERA };
}

uint8_t MockCameraDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

MockCameraDriver::MockCameraDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);
}

unsigned int MockCameraDriver::getRate()
{
  return 20; // 20 Hz as default spin rate to surpass AVT vimba driver rate of 19Hz
}

int MockCameraDriver::onRun()
{
  addPassthroughPub<sensor_msgs::CameraInfo>(bag_prefix_ + camera_info_topic_, camera_info_topic_, false, 10);
  addPassthroughPub<sensor_msgs::Image>(bag_prefix_ + image_raw_topic_, image_raw_topic_, false, 10);
  addPassthroughPub<sensor_msgs::Image>(bag_prefix_ + image_rects_topic_, image_rects_topic_, false, 10);
  addPassthroughPub<autoware_msgs::ProjectionMatrix>(bag_prefix_ + projection_matrix_topic_, projection_matrix_topic_,
                                                     false, 10);

  return 0;
}

}  // namespace mock_drivers