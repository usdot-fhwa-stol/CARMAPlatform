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
struct PlatoonPluginConfig
{
  // following parameters are for general platooning plugin
  double vehicleLength         = 5.0;  // m
  int    maxPlatoonSize        = 10;   // 1
  int    algorithmType         = 0;    // N/A
  int    statusMessageInterval = 100;  // ms
  int    infoMessageInterval   = 200;  // ms
  double mvr_duration          = 15;   // s
  double config_limit          = 0.0;  // m/s
  double epislon              = 0.001; // m/s
  
  // following parameters are for platoon forming and operation
  double timeHeadway           = 2.0;  // s
  double standStillHeadway     = 12.0; // m
  double maxAllowedJoinTimeGap = 15.0; // s
  double maxAllowedJoinGap     = 90.0; // m
  double desiredJoinTimeGap    = 4.0;  // s
  double desiredJoinGap        = 30.0; // m
  double waitingStateTimeout   = 25.0; // s
  double cmdSpeedMaxAdjustment = 10.0; // m/s
    
  // following parameters are mainly for APF leader selection
  double lowerBoundary         = 1.6;  // s
  double upperBoundary         = 1.7;  // s
  double maxSpacing            = 4.0;  // s
  double minSpacing            = 3.9;  // s
  double minGap                = 22.0; // m
  double maxGap                = 32.0; // m
  double maxCrosstrackError    = 2.0;  // m

  std::string vehicle_id       = "default_id";


  friend std::ostream& operator<<(std::ostream& output, const PlatoonPluginConfig& c)
  {
    output << "PlatoonPluginConfig { " << std::endl
          << "maxPlatoonSize: " << c.maxPlatoonSize << std::endl
          << "algorithmType: " << c.algorithmType << std::endl
          << "statusMessageInterval: " << c.statusMessageInterval << std::endl
          << "infoMessageInterval: " << c.infoMessageInterval << std::endl
          << "timeHeadway: " << c.timeHeadway << std::endl
          << "standStillHeadway: " << c.standStillHeadway << std::endl
          << "maxAllowedJoinTimeGap: " << c.maxAllowedJoinTimeGap << std::endl
          << "desiredJoinTimeGap: " << c.desiredJoinTimeGap << std::endl
          << "desiredJoinGap: " << c.desiredJoinGap << std::endl
          << "waitingStateTimeout: " << c.waitingStateTimeout << std::endl
          << "cmdSpeedMaxAdjustment: " << c.cmdSpeedMaxAdjustment << std::endl
          << "lowerBoundary: " << c.lowerBoundary << std::endl
          << "upperBoundary: " << c.upperBoundary << std::endl
          << "maxSpacing: " << c.maxSpacing << std::endl
          << "minSpacing: " << c.minSpacing << std::endl
          << "kpminGapPID: " << c.minGap << std::endl
          << "maxGap: " << c.maxGap << std::endl
          << "maxCrosstrackError: " << c.maxCrosstrackError << std::endl
          << "vehicle_id: " << c.vehicle_id << std::endl
          << "}" << std::endl;
    return output;
  }
};
