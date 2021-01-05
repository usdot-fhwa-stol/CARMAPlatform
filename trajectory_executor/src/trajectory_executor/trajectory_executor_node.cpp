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

#include <ros/ros.h>
#include "trajectory_executor/trajectory_executor.hpp"

/**
 * Main application entry point for TrajectoryExecutor Node
 */
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "trajectory_executor");
  ROS_DEBUG("Entered main function for Trajectory executor, starting node...");
  trajectory_executor::TrajectoryExecutor te;

  te.init();
  te.run();

  return 0;
}
