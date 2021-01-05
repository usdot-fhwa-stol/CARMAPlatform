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

namespace mock_drivers
{
template <typename T>
void MockDriver::addPassthroughPub(const std::string& sub_topic, const std::string& pub_topic, bool latch,
                                   size_t queue_size)
{
  // Create pointers for publishers
  ROSCommsPtr<T> pub_ptr = boost::make_shared<ROSComms<T>>(CommTypes::pub, latch, queue_size, pub_topic);

  mock_driver_node_.addPub(pub_ptr);

  std::function<void(ConstPtrRef<T>)> callback = std::bind(
      [&](ConstPtrRef<T> in) {
        T out = *in;
        out.header.stamp = ros::Time::now(); // Update message timestamp
        mock_driver_node_.publishData<const T&>(pub_topic, out);
      },
      std::placeholders::_1);

  ConstPtrRefROSCommsPtr<T> outbound_sub_ptr_ =
      boost::make_shared<ConstPtrRefROSComms<T>>(callback, CommTypes::sub, false, queue_size, sub_topic);

  mock_driver_node_.addSub(outbound_sub_ptr_);
}

template <typename T>
void MockDriver::addPassthroughPubNoHeader(const std::string& sub_topic, const std::string& pub_topic, bool latch,
                                           size_t queue_size)
{
  // Create pointers for publishers
  ROSCommsPtr<T> pub_ptr = boost::make_shared<ROSComms<T>>(CommTypes::pub, latch, queue_size, pub_topic);

  mock_driver_node_.addPub(pub_ptr);

  std::function<void(ConstPtrRef<T>)> callback = std::bind(
      [&](ConstPtrRef<T> in) {
        T out = *in;
        mock_driver_node_.publishDataNoHeader<const T&>(pub_topic, out);
      },
      std::placeholders::_1);

  ConstPtrRefROSCommsPtr<T> outbound_sub_ptr_ =
      boost::make_shared<ConstPtrRefROSComms<T>>(callback, CommTypes::sub, false, queue_size, sub_topic);

  mock_driver_node_.addSub(outbound_sub_ptr_);
}

}  // namespace mock_drivers