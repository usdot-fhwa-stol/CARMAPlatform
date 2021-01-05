/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <functional>

#include "RoadwayObjectsWorker.h"

namespace objects
{
class RoadwayObjectsNode
{
private:
  // node handle
  ros::CARMANodeHandle nh_;

  // subscriber
  ros::Subscriber external_objects_sub_;

  // publisher
  ros::Publisher roadway_obs_pub_;

  // World Model Listener. Must be declared before object_worker_ for proper initialization.
  carma_wm::WMListener wm_listener_;

  // Worker class object
  RoadwayObjectsWorker object_worker_;
public:
  /*!
    \brief RoadwayObjectsNode constructor
  */
  RoadwayObjectsNode();

  /*!
    \brief Callback to publish RoadwayObstacleList
  */
  void publishObstacles(const cav_msgs::RoadwayObstacleList& obs_list);

  /*!
    \brief General starting point to run this node
  */
  void run();
};

}  // namespace objects
