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

#include <ros/ros.h>

#include <rosbag_mock_drivers/MockCameraDriver.h>
#include <rosbag_mock_drivers/MockCANDriver.h>
#include <rosbag_mock_drivers/MockCommsDriver.h>
#include <rosbag_mock_drivers/MockControllerDriver.h>
#include <rosbag_mock_drivers/MockGNSSDriver.h>
#include <rosbag_mock_drivers/MockIMUDriver.h>
#include <rosbag_mock_drivers/MockLidarDriver.h>
#include <rosbag_mock_drivers/MockRadarDriver.h>
#include <rosbag_mock_drivers/MockRoadwaySensorDriver.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mock_driver");

  if (strcmp("camera", argv[1]) == 0)
  {
    mock_drivers::MockCameraDriver node;
    node.run();
  }

  else if (strcmp("can", argv[1]) == 0)
  {
    mock_drivers::MockCANDriver node;
    node.run();
  }

  else if (strcmp("comms", argv[1]) == 0)
  {
    mock_drivers::MockCommsDriver node;
    node.run();
  }

  else if (strcmp("controller", argv[1]) == 0)
  {
    mock_drivers::MockControllerDriver node;
    node.run();
  }

  else if (strcmp("gnss", argv[1]) == 0)
  {
    mock_drivers::MockGNSSDriver node;
    node.run();
  }

  else if (strcmp("imu", argv[1]) == 0)
  {
    mock_drivers::MockIMUDriver node;
    node.run();
  }

  else if (strcmp("lidar", argv[1]) == 0)
  {
    mock_drivers::MockLidarDriver node;
    node.run();
  }

  else if (strcmp("radar", argv[1]) == 0)
  {
    mock_drivers::MockRadarDriver node;
    node.run();
  }

  else if (strcmp("roadway_sensor", argv[1]) == 0)
  {
    mock_drivers::MockRoadwaySensorDriver node;
    node.run();
  }

  return 0;
}