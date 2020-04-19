#pragma once

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

#include <vector>
#include <ros/ros.h>
#include <stdint.h>
#include <algorithm>

namespace bsm_generator
{
    class BSMGeneratorWorker
    {
        public:
            
            BSMGeneratorWorker();
            BSMGeneratorWorker(uint8_t msg_count);
            uint8_t getNextMsgCount();
            std::vector<uint8_t> getMsgId(const ros::Time now);
            uint16_t getSecMark(const ros::Time now);
            float getSpeedInRange(const double speed);
            float getSteerWheelAngleInRnage(const double angle);
            float getLongAccelInRange(const float accel);
            float getYawRateInRange(const double yaw_rate);
            uint8_t getBrakeAppliedStatus(const double brake);
            float getHeadingInRange(const float heading);

        private:

            // variables for BSM generation
            uint8_t msg_count_;
            int random_id_;
            ros::Time last_id_generation_time_;
    };
}
