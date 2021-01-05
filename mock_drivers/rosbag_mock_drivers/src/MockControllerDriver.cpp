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

#include "rosbag_mock_drivers/MockControllerDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockControllerDriver::getDriverTypes()
{
  return { DriverType::CONTROLLER };
}

uint8_t MockControllerDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

void MockControllerDriver::vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg)
{
  robot_enabled_ = true;  // If a command was received set the robot enabled status to true
}

bool MockControllerDriver::enableRoboticSrv(const cav_srvs::SetEnableRobotic::Request& req,
                                            cav_srvs::SetEnableRobotic::Response& res)
{
  if (robot_enabled_ && req.set == cav_srvs::SetEnableRobotic::Request::ENABLE)
  {
    robot_active_ = true;
  }
  else
  {
    robot_active_ = false;
  }

  return true;
}

MockControllerDriver::MockControllerDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);

  robot_status_ptr_ =
      boost::make_shared<ROSComms<cav_msgs::RobotEnabled>>(CommTypes::pub, false, 10, robot_status_topic_);

  vehicle_cmd_ptr_ = boost::make_shared<ConstPtrRefROSComms<autoware_msgs::VehicleCmd>>(
      std::bind(&MockControllerDriver::vehicleCmdCallback, this, std::placeholders::_1), CommTypes::sub, false, 10,
      vehicle_cmd_topic_);

  enable_robotic_ptr_ =
      boost::make_shared<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>(
          std::bind(&MockControllerDriver::enableRoboticSrv, this, std::placeholders::_1, std::placeholders::_2),
          CommTypes::srv, enable_robotic_srv_);
}

bool MockControllerDriver::onSpin()
{
  cav_msgs::RobotEnabled robot_status;
  robot_status.robot_active = robot_active_;
  robot_status.robot_enabled = robot_enabled_;

  mock_driver_node_.publishDataNoHeader<cav_msgs::RobotEnabled>(robot_status_topic_, robot_status);

  return true;
}

unsigned int MockControllerDriver::getRate()
{
  return 20;  // 20 Hz as default spin rate to match expected controller data rate
}

int MockControllerDriver::onRun()
{
  // driver publisher, subscriber, and service
  mock_driver_node_.addPub(robot_status_ptr_);
  mock_driver_node_.addSub(vehicle_cmd_ptr_);
  mock_driver_node_.addSrv(enable_robotic_ptr_);

  return 0;
}

}  // namespace mock_drivers