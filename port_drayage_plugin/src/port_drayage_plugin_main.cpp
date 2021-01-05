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

// @SONAR_STOP@
#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <memory>
#include "port_drayage_plugin/port_drayage_plugin.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "port_drayage_plugin");
    std::shared_ptr<ros::CARMANodeHandle> nh = std::make_shared<ros::CARMANodeHandle>("");
    std::shared_ptr<ros::CARMANodeHandle> pnh = std::make_shared<ros::CARMANodeHandle>("~");
    port_drayage_plugin::PortDrayagePlugin pdp{nh, pnh};
    return pdp.run();
}

// @SONAR_START