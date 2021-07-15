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
 * \brief Stuct containing the algorithm configuration values for the PlatooningControlPlugin
 */
struct PlatooningControlPluginConfig
{
  double timeHeadway = 6.0;
  double standStillHeadway = 12.0;
  double maxAccel = 2.5;
  double Kp = 0.5;
  double Kd = -0.5;
  double Ki = 0.0;
  double max_value = 100;
  double min_value = -100;
  double dt = 0.1;
  double adjustmentCap = 10;
  int CMD_TIMESTEP = 100;
  double integratorMax = 100;
  double integratorMin = -100;
  double Kdd = 4.5;                             //coeficient for smooth steering
  double wheelbase = 2.7;
  double lowpass_gain = 0.5;
  double lookahead_ratio = 2.0;
  double min_lookahead_dist = 6.0;
  std::string vehicle_id = "DEFAULT_VEHICLE_ID";         // Vehicle id is the license plate of the vehicle
  
  
  friend std::ostream& operator<<(std::ostream& output, const PlatooningControlPluginConfig& c)
  {
    output << "InLaneCruisingPluginConfig { " << std::endl
           << "timeHeadway: " << c.timeHeadway << std::endl
           << "standStillHeadway: " << c.standStillHeadway << std::endl
           << "maxAccel: " << c.maxAccel << std::endl
           << "Kp: " << c.Kp << std::endl
           << "Kd: " << c.Kd << std::endl
           << "Ki: " << c.Ki << std::endl
           << "max_value: " << c.max_value << std::endl
           << "min_value: " << c.min_value << std::endl
           << "dt: " << c.dt << std::endl
           << "adjustmentCap: " << c.adjustmentCap << std::endl
           << "CMD_TIMESTEP: " << c.CMD_TIMESTEP << std::endl
           << "integratorMax: " << c.integratorMax << std::endl
           << "integratorMin: " << c.integratorMin << std::endl
           << "Kdd: " << c.Kdd << std::endl
           << "wheelbase: " << c.wheelbase << std::endl
           << "lowpass_gain: " << c.lowpass_gain << std::endl
           << "lookahead_ratio: " << c.lookahead_ratio << std::endl
           << "min_lookahead_dist: " << c.min_lookahead_dist << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
    return output;
  }
};