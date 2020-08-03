/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#ifndef EXTERNAL_OBJECT_H
#define EXTERNAL_OBJECT_H

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <functional>
#include "motion_computation_worker.h"

namespace object{

class MotionComputationNode
{

 private:
  
  //node handle
  ros::CARMANodeHandle nh_;
  ros::CARMANodeHandle pnh_;
   
  //subscriber
  ros::Subscriber motion_comp_sub_;

  //publisher
  ros::Publisher carma_obj_pub_;
  
  //ObjectDetectionTrackingWorker class object
  MotionComputationWorker motion_worker_;
  
    /*!fn initialize()
  \brief initialize this node before running
  */
    void initialize();

 public:
  
   /*! \fn MotionComputationNode()
    \brief MotionComputationNode constructor 
   */
  MotionComputationNode();

     /*! \fn publishObject()
    \brief Callback to publish ObjectList
   */
  void publishObject(const cav_msgs::ExternalObjectList& obj_pred_msg);

  /*!fn run()
  \brief General starting point to run this node
  */
  void run();
  
};

}//object

#endif /* EXTERNAL_OBJECT_H */
