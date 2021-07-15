#pragma once

/*
 * Copyright (C) 2020 LEIDOS.
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
 * \brief Stuct containing the algorithm configuration values for the PlatooningTacticalPlugin
 */
struct PlatooningTacticalPluginConfig
{
  double trajectory_time_length = 6.0;     // Trajectory length in seconds
  double curve_resample_step_size = 1.0;   // Curve re-sampling step size in m
  int default_downsample_ratio = 36.0;              // Amount to downsample input lanelet centerline data.
  int turn_downsample_ratio = 20.0;              // Amount to downsample input lanelet centerline data if the lanelet is marked as a turn
                                           // Corresponds to saving each nth point.
  double minimum_speed = 2.2352;           // Minimum allowable speed in m/s
  double max_accel = 3;                  // Maximum allowable longitudinal acceleration in m/s^2
  double max_accel_multiplier = 0.85;     // Multiplier of max_accel to bring the value under max_accel
  double lat_accel_multiplier = 0.50;      // Multiplier of lat_accel to bring the value under lat_accel TODO: needs to be tuned
  double lateral_accel_limit = 2.5;        // Maximum allowable lateral acceleration m/s^2
  int speed_moving_average_window_size = 5;      // Size of the window used in the moving average filter to smooth both the speed profile
  int curvature_moving_average_window_size = 9;  // Size of the window used in the moving average filter to smooth the curvature profile
                                           // computed curvature and output speeds
  double back_distance = 20;               // Number of meters behind the first maneuver that need to be included in points for curvature calculation
  bool enable_object_avoidance = true;    // Activate object avoidance logic
  bool publish_debug = false; // True if debug publishing will be enabled
  double buffer_ending_downtrack = 20.0;
  
  friend std::ostream& operator<<(std::ostream& output, const PlatooningTacticalPluginConfig& c)
  {
    output << "PlatooningTacticalPluginConfig { " << std::endl
           << "trajectory_time_length: " << c.trajectory_time_length << std::endl
           << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
           << "default_downsample_ratio: " << c.default_downsample_ratio << std::endl
           << "turn_downsample_ratio: " << c.turn_downsample_ratio << std::endl
           << "minimum_speed: " << c.minimum_speed << std::endl
           << "max_accel: " << c.max_accel << std::endl
           << "max_accel_multiplier: " << c.max_accel_multiplier << std::endl
           << "lat_accel_multiplier: " << c.lat_accel_multiplier << std::endl
           << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
           << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
           << "curvature_moving_average_window_size: " << c.curvature_moving_average_window_size << std::endl
           << "back_distance: " << c.back_distance << std::endl
           << "enable_object_avoidance: " << c.enable_object_avoidance << std::endl
           << "publish_debug: " << c.publish_debug << std::endl
           << "buffer_ending_downtrack: " << c.buffer_ending_downtrack << std::endl
           << "}" << std::endl;
    return output;
  }
};