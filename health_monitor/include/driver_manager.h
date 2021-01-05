#pragma once
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

#include <cav_msgs/DriverStatus.h>
#include <cav_msgs/SystemAlert.h>
#include "entry_manager.h"
#include <iostream>

namespace health_monitor
{
    class DriverManager
    {
        public:
            
            /*!
             * \brief Default constructor for DriverManager with driver_timeout_ = 1000
             */
            DriverManager();
            /*!
             * \brief Constructor for DriverManager takes in crtitical driver names and driver timeout
             */
            DriverManager(std::vector<std::string> critical_driver_names, const long driver_timeout,std::vector<std::string> lidar_gps_driver_names);
            /*!
             * \brief Update driver status
             */
            void update_driver_status(const cav_msgs::DriverStatusConstPtr& msg, long current_time);
            /*!
             * \brief Check if all critical drivers are operational for truck
             */
            std::string are_critical_drivers_operational_truck(long current_time);
            /*!
             * \brief Check if all critical drivers are operational for car
             */
            std::string are_critical_drivers_operational_car(long current_time);
            /*!
             * \brief Evaluate if the sensor is available
             */
            void evaluate_sensor(int &sensor_input,bool available,long current_time,long timestamp,long driver_timeout);
            /*!
             * \brief Handle the spin and publisher
             */
            cav_msgs::SystemAlert handleSpin(bool truck,bool car,long time_now,long start_up_timestamp,long startup_duration);

        private:

            EntryManager em_;
            bool starting_up_ = true;
            // timeout for critical driver timeout
            long driver_timeout_ {1000};

    };
}