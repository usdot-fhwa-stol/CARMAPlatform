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
#include "roadway_objects/RoadwayObjectsNode.h"

namespace objects
{
using std::placeholders::_1;

RoadwayObjectsNode::RoadwayObjectsNode()
  : object_worker_(wm_listener_.getWorldModel(), std::bind(&RoadwayObjectsNode::publishObstacles, this, _1))
{
  external_objects_sub_ =
      nh_.subscribe("external_objects", 10, &RoadwayObjectsWorker::externalObjectsCallback, &object_worker_);
  roadway_obs_pub_ = nh_.advertise<cav_msgs::RoadwayObstacleList>("roadway_objects", 10);
}

void RoadwayObjectsNode::publishObstacles(const cav_msgs::RoadwayObstacleList& obs_msg)
{
  roadway_obs_pub_.publish(obs_msg);
}

void RoadwayObjectsNode::run()
{
  nh_.setSpinRate(20);
  nh_.spin();
}

}  // namespace objects
