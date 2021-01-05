#pragma once
/*
 * Copyright (C) 2020-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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
#include <thread>
#include <chrono>

namespace testing
{
/*!
 * \brief Waits for the specified number of subscribers to exist on the topic
 * before continuing. Will wait until num_subscribers exist or until timeout_millis
 * has elapsed.
 *
 * \param pub The publisher of the topic you are waiting for
 * \param num_subscribers The target number of subscribers to achieve
 * \param timeout_millis The maximum number of milliseconds to wait
 *
 * \return false if the timeout expires; true otherwise
 */
bool waitForSubscribers(ros::Publisher pub, int num_subscribers, int timeout_millis)
{
  int elapsed_millis = 0;
  while (pub.getNumSubscribers() < num_subscribers && elapsed_millis < timeout_millis)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    elapsed_millis += 100;
  }

  if (elapsed_millis > timeout_millis && pub.getNumSubscribers() < num_subscribers)
  {
    ROS_WARN_STREAM("Target number of subscribers for " << pub.getTopic() << " not reached due to timeout! ("
                                                        << pub.getNumSubscribers() << "/" << num_subscribers
                                                        << " subscribers)");
    return false;
  }

  return true;
}

}  // namespace testing