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
 * \brief Stuct containing the algorithm configuration values for the YieldPluginConfig
 */
struct YieldPluginConfig
{
  double acceleration_adjustment_factor = 4.0;  // Adjustment factor for safe and comfortable acceleration/deceleration 
  double collision_horizon = 10.0;        // time horizon for collision detection in s
  double min_obstacle_speed = 2.0;       // Minimum speed for moving obstacle in m/s
  double tpmin = 2.0;                     // minimum object avoidance planning time in s
  double yield_max_deceleration = 3.0;  // max deceleration value in m/s^2
  double x_gap = 2.0;                       // minimum safety gap in m
  double vehicle_length = 5.0;              // Host vehicle length in m
  double vehicle_height = 3.0;              // Host vehicle height in m
  double vehicle_width = 2.5;              // Host vehicle width in m
  double max_stop_speed = 1.0;           //  Maximum speed value to consider the ego vehicle stopped in m/s


  friend std::ostream& operator<<(std::ostream& output, const YieldPluginConfig& c)
  {
    output << "YieldPluginConfig { " << std::endl
          << "acceleration_adjustment_factor: " << c.acceleration_adjustment_factor << std::endl
          << "collision_horizon: " << c.collision_horizon << std::endl
          << "min_obstacle_speed: " << c.min_obstacle_speed << std::endl
          << "yield_max_deceleration: " << c.yield_max_deceleration << std::endl
          << "x_gap: " << c.x_gap << std::endl
          << "vehicle_length: " << c.vehicle_length << std::endl
          << "vehicle_height: " << c.vehicle_height << std::endl
          << "vehicle_width: " << c.vehicle_width << std::endl
          << "max_stop_speed: " << c.max_stop_speed << std::endl
          << "}" << std::endl;
    return output;
  }
};
