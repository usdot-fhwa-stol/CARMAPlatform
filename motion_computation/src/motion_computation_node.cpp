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
#include <motion_computation_node.h>
#include <motion_computation_worker.h>

namespace object{

  using std::placeholders::_1;

  MotionComputationNode::MotionComputationNode(): pnh_("~"), motion_worker_(std::bind(&MotionComputationNode::publishObject, this, _1)){};

  void MotionComputationNode::initialize()
  {
    // Load parameters
    double step = 0.1;
    double period = 2.0;
    double ax = 9.0;
    double ay = 9.0;
    double process_noise_max = 1000.0;
    double drop_rate = 0.9;

    pnh_.param<double>("prediction_time_step", step, step);
    pnh_.param<double>("prediction_period", period, period);
    pnh_.param<double>("cv_x_accel_noise", ax, ax);
    pnh_.param<double>("cv_y_accel_noise", ay, ay);
    pnh_.param<double>("prediction_process_noise_max", process_noise_max, process_noise_max);
    pnh_.param<double>("prediction_confidence_drop_rate", drop_rate, drop_rate);

    motion_worker_.setPredictionTimeStep(step);
    motion_worker_.setPredictionPeriod(period);
    motion_worker_.setXAccelerationNoise(ax);
    motion_worker_.setYAccelerationNoise(ay);
    motion_worker_.setProcessNoiseMax(process_noise_max);
    motion_worker_.setConfidenceDropRate(drop_rate);

    // Setup pub/sub
    motion_comp_sub_=nh_.subscribe("external_objects",1,&MotionComputationWorker::predictionLogic,&motion_worker_);
    carma_obj_pub_=nh_.advertise<cav_msgs::ExternalObjectList>("external_object_predictions", 2);
  }

  void MotionComputationNode::publishObject(const cav_msgs::ExternalObjectList& obj_pred_msg)
  {
  carma_obj_pub_.publish(obj_pred_msg);
  }

  void MotionComputationNode::run()
  {
    initialize();
    nh_.setSpinRate(20);
    nh_.spin();
  }

}//object
