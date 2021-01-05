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

#include "rosbag_mock_drivers/MockCommsDriver.h"

namespace mock_drivers
{
std::vector<DriverType> MockCommsDriver::getDriverTypes()
{
  return { DriverType::COMMS };
}

uint8_t MockCommsDriver::getDriverStatus()
{
  return cav_msgs::DriverStatus::OPERATIONAL;
}

void MockCommsDriver::outboundCallback(const cav_msgs::ByteArray::ConstPtr& msg) const
{
  ROS_DEBUG_STREAM("Received Byte Array of type: " << msg->messageType);
};

MockCommsDriver::MockCommsDriver(bool dummy)
{
  mock_driver_node_ = MockDriverNode(dummy);

  std::function<void(const cav_msgs::ByteArray::ConstPtr&)> outbound_ptr =
      std::bind(&MockCommsDriver::outboundCallback, this, std::placeholders::_1);
  outbound_sub_ptr_ = boost::make_shared<ConstPtrRefROSComms<cav_msgs::ByteArray>>(outbound_ptr, CommTypes::sub, false,
                                                                                   10, outbound_binary_topic_);
}

unsigned int MockCommsDriver::getRate()
{
  return 20; // 20 Hz as default spin rate to match expected comms data rate
}

int MockCommsDriver::onRun()
{
  // driver publisher and subscriber
  addPassthroughPub<cav_msgs::ByteArray>(bag_prefix_ + inbound_binary_topic_, inbound_binary_topic_, false, 10);

  mock_driver_node_.addSub(outbound_sub_ptr_);

  return 0;
}

}  // namespace mock_drivers