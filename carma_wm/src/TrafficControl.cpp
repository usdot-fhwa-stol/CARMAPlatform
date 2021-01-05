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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <carma_wm/TrafficControl.h>

namespace carma_wm
{

void toBinMsg(std::shared_ptr<carma_wm::TrafficControl> gf_ptr, autoware_lanelet2_msgs::MapBin* msg)
{
  if (msg == nullptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": msg is null pointer!");
    return;
  }
  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *gf_ptr;
  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void fromBinMsg(const autoware_lanelet2_msgs::MapBin& msg, std::shared_ptr<carma_wm::TrafficControl> gf_ptr)
{
  if (!gf_ptr)
  {
    ROS_ERROR_STREAM(__FUNCTION__ << ": gf_ptr is null pointer!");
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());
  
  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive oa(ss);
  oa >> *gf_ptr;
}

}  // namespace carma_wm
