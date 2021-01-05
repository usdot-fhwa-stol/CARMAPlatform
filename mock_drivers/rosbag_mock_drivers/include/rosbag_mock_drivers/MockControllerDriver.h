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
#include <autoware_msgs/VehicleCmd.h>
#include <cav_msgs/RobotEnabled.h>
#include <cav_srvs/SetEnableRobotic.h>

namespace mock_drivers
{
/*! \brief Mock Controller driver. Implements controller service and feedback logic to support CARMA Platform Guidance State Machine */
class MockControllerDriver : public MockDriver
{
private:
  boost::shared_ptr<ROSComms<cav_msgs::RobotEnabled>> robot_status_ptr_;
  ConstPtrRefROSCommsPtr<autoware_msgs::VehicleCmd> vehicle_cmd_ptr_;
  boost::shared_ptr<ROSComms<cav_srvs::SetEnableRobotic::Request&, cav_srvs::SetEnableRobotic::Response&>>
      enable_robotic_ptr_;

  // Pub
  const std::string robot_status_topic_ = "controller/robot_status";

  // Sub
  const std::string vehicle_cmd_topic_ = "vehicle_cmd";
  const std::string enable_robotic_srv_ = "controller/enable_robotic";

  // Robot Status flags
  bool robot_active_ = false;
  bool robot_enabled_ = false;

protected:
  int onRun() override;

public:

  /**
   * \brief Callback for the vehicle command topic. This callback must be triggered at least once before the enable robotic service is called.
   * 
   * \param msg The vehicle command to receive
   */ 
  void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg);

  /**
   * \brief Callback for the enable robotic service. Triggering this callback will modify the RobotEnabled message output by this driver.
   * 
   * \param req The service request in message
   * \param res The service response out message
   * 
   * \return Flag idicating if the service was processed successfully. 
   */ 
  bool enableRoboticSrv(const cav_srvs::SetEnableRobotic::Request& req, cav_srvs::SetEnableRobotic::Response& res);

  // Overrides
  MockControllerDriver(bool dummy = false);
  ~MockControllerDriver() {};
  bool onSpin() override;
  std::vector<DriverType> getDriverTypes() override;
  uint8_t getDriverStatus() override;
  unsigned int getRate() override;
};

}  // namespace mock_drivers