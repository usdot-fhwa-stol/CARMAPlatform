#pragma once

/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <iostream>

/**
 * \brief Stuct containing the algorithm configuration values for the InLaneCruisingPlugin
 */
struct InLaneCruisingPluginConfig
{
  double trajectory_time_length = 6.0;     // Trajectory length in seconds
  double curve_resample_step_size = 1.0;   // Curve re-sampling step size in m
  int downsample_ratio = 8.0;              // Amount to downsample input lanelet centerline data.
                                           // Corresponds to saving each nth point.
  double minimum_speed = 2.2352;           // Minimum allowable speed in m/s
  double max_accel = 1.5;                  // Maximum allowable longitudinal acceleration in m/s^2
  
  double minimum_lookahead_distance = 5.0; // Minimum value for lookahead distance in m
  double maximum_lookahead_distance = 25.0;// Maximum value for lookahead distance in m
  double minimum_lookahead_speed = 2.8;    // Minimum speed value for lookahead calculation in m/s
  double maximum_lookahead_speed = 13.9;   // Maximum speed value for lookahead calculation in m/s
  double lookahead_ratio = 2.0;            // ratio to calculate lookahead distance from speed
  

  double lateral_accel_limit = 1.5;        // Maximum allowable lateral acceleration m/s^2
  int moving_average_window_size = 5;      // Size of the window used in the moving average filter to smooth both the
                                           // computed curvature and output speeds
  int curvature_calc_lookahead_count = 1;  // Number of points to look ahead when calculating the curvature
                                           // of the lanelet centerline

  friend std::ostream& operator<<(std::ostream& output, const InLaneCruisingPluginConfig& c)
  {
    output << "InLaneCruisingPluginConfig { " << std::endl
           << "trajectory_time_length: " << c.trajectory_time_length << std::endl
           << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
           << "downsample_ratio: " << c.downsample_ratio << std::endl
           << "minimum_speed: " << c.minimum_speed << std::endl
           << "max_accel: " << c.max_accel << std::endl
           << "minimum_lookahead_distance: " << c.minimum_lookahead_distance << std::endl
           << "maximum_lookahead_distance: " << c.maximum_lookahead_distance << std::endl
           << "minimum_lookahead_speed: " << c.minimum_lookahead_speed << std::endl
           << "maximum_lookahead_speed: " << c.maximum_lookahead_speed << std::endl
           << "lookahead_ratio: " << c.lookahead_ratio << std::endl
           << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
           << "moving_average_window_size: " << c.moving_average_window_size << std::endl
           << "curvature_calc_lookahead_count: " << c.curvature_calc_lookahead_count << std::endl
           << "}" << std::endl;
    return output;
  }
};