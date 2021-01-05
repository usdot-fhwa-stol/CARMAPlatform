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
#include "object_detection_tracking_node.h"

namespace object{

  using std::placeholders::_1;

  ObjectDetectionTrackingNode::ObjectDetectionTrackingNode(): pnh_("~"), object_worker_(std::bind(&ObjectDetectionTrackingNode::publishObject, this, _1)){}

  void ObjectDetectionTrackingNode::initialize()
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

    object_worker_.setPredictionTimeStep(step);
    object_worker_.setPredictionPeriod(period);
    object_worker_.setXAccelerationNoise(ax);
    object_worker_.setYAccelerationNoise(ay);
    object_worker_.setProcessNoiseMax(process_noise_max);
    object_worker_.setConfidenceDropRate(drop_rate);

    // Setup pub/sub
    autoware_obj_sub_=nh_.subscribe("detected_objects",10,&ObjectDetectionTrackingWorker::detectedObjectCallback,&object_worker_);
    carma_obj_pub_=nh_.advertise<cav_msgs::ExternalObjectList>("external_objects", 10);
  }

  void ObjectDetectionTrackingNode::publishObject(const cav_msgs::ExternalObjectList& obj_msg)
  {
  carma_obj_pub_.publish(obj_msg);
  }

  void ObjectDetectionTrackingNode::run()
  {
    initialize();
    nh_.setSpinRate(20);
    nh_.spin();
  }

}//object
