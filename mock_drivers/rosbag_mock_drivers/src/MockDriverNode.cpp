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

#include "rosbag_mock_drivers/MockDriverNode.h"

namespace mock_drivers
{
MockDriverNode::MockDriverNode()
{
}
MockDriverNode::MockDriverNode(bool dummy) : dummy_(dummy)
{
}

std::string MockDriverNode::getGraphName() const
{
  if (dummy_)
  {
    return "dummy_mock_driver";
  }

  return ros::this_node::getName();
}

void MockDriverNode::spin(double rate) const
{
  if (!dummy_)
  {
    ros::CARMANodeHandle::setSpinRate(rate);
    ros::CARMANodeHandle::spin();
  }
}

void MockDriverNode::setSpinCallback(std::function<bool()> cb) const
{
  if (!dummy_)
  {
    ros::CARMANodeHandle::setSpinCallback(cb);
  }
}

void MockDriverNode::init()
{
  if (!dummy_)
  {
    cnh_ = boost::make_shared<ros::CARMANodeHandle>(ros::CARMANodeHandle());
  }
}

}  // namespace mock_drivers