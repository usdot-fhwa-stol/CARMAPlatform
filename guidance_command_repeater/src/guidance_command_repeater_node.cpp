/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include <sstream>
#include <cav_msgs/SpeedAccel.h>

#include <ros/ros.h>
#include "guidance_command_repeater/GuidanceCommandRepeater.hpp"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "guidance_command_repeater");
  ros::NodeHandle nodeHandle("~");
  guidance_command_repeater::GuidanceCommandRepeater GuidanceCommandRepeater(nodeHandle);

  ros::Rate rate(GuidanceCommandRepeater.rate);

  while (ros::ok())
  {
    GuidanceCommandRepeater.publisher();
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}

