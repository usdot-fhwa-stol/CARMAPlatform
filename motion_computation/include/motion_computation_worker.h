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

#ifndef EXTERNAL_OBJECT_WORKER_H
#define EXTERNAL_OBJECT_WORKER_H

#include <ros/ros.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/ExternalObjectList.h>
#include <functional>

namespace object{

class MotionComputationWorker
{

 public:

  using PublishObjectCallback = std::function<void(const cav_msgs::ExternalObjectList&)>;

  /*!
   * \brief Constructor
   */

  MotionComputationWorker(const PublishObjectCallback& obj_pub);
    
    /*! \fn predictionLogic(cav_msgs::ExternalObjectListPtr obj_list)
    \brief predictionLogic populates duplicated detected object along with its velocity,yaw, yaw_rate and static/dynamic class to ExternalObject message.
    \param  msg array of detected objects.
    */
  void predictionLogic(cav_msgs::ExternalObjectListPtr obj_list);

  // Setters for the prediction parameters
  void setPredictionTimeStep(double time_step);
  void setPredictionPeriod(double period);
  void setXAccelerationNoise(double noise);
  void setYAccelerationNoise(double noise);
  void setProcessNoiseMax(double noise_max);
  void setConfidenceDropRate(double drop_rate);
 
 private:

  // local copy of external object publihsers

  PublishObjectCallback obj_pub_;

  // Prediction parameters
  double prediction_time_step_ = 0.1;
  double prediction_period_ = 2.0;
  double cv_x_accel_noise_ = 9.0;
  double cv_y_accel_noise_ = 9.0;
  double prediction_process_noise_max_ = 1000.0;
  double prediction_confidence_drop_rate_ = 0.9;
  
};

}//object

#endif /* EXTERNAL_OBJECT_WORKER_H */
