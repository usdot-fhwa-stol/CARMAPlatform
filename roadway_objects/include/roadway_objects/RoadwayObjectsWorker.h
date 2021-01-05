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
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <carma_wm/WorldModel.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/RoadwayObstacle.h>
#include <functional>

namespace objects
{
class RoadwayObjectsWorker
{
public:
  using PublishObstaclesCallback = std::function<void(const cav_msgs::RoadwayObstacleList&)>;

  /*!
   * \brief Constructor
   */
  RoadwayObjectsWorker(carma_wm::WorldModelConstPtr wm, PublishObstaclesCallback obj_pub);

  /*!
    \brief Converts the provided ExternalObjectList in a RoadwayObstacleList and republishes it

    \param msg array of detected objects.
  */
  void externalObjectsCallback(const cav_msgs::ExternalObjectListConstPtr& msg);

private:
  // local copy of external object publihsers

  PublishObstaclesCallback obj_pub_;

  carma_wm::WorldModelConstPtr wm_;
};

}  // namespace objects
