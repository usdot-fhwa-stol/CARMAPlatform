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

#include "driver_manager.h"

namespace health_monitor
{

    DriverManager::DriverManager() {}

    DriverManager::DriverManager(std::vector<std::string> critical_driver_names, const long driver_timeout, std::vector<std::string> lidar_gps_driver_names):
                                em_(EntryManager(critical_driver_names,lidar_gps_driver_names)), driver_timeout_(driver_timeout) {}

    void DriverManager::update_driver_status(const cav_msgs::DriverStatusConstPtr& msg, long current_time)
    {
        Entry driver_status(msg->status == cav_msgs::DriverStatus::OPERATIONAL || msg->status == cav_msgs::DriverStatus::DEGRADED,true, msg->name, current_time, 0, "");
        em_.update_entry(driver_status);
    }

    void DriverManager::evaluate_sensor(int &sensor_input,bool available,long current_time,long timestamp,long driver_timeout)
    {
        if((!available) || (current_time-timestamp > driver_timeout))
        {
            sensor_input=0;
        }
        else
        {
            sensor_input=1;
        }
    }

    std::string DriverManager::are_critical_drivers_operational_truck(long current_time)
    {
        int ssc=0;
        int lidar1=0;
        int lidar2=0;
        int gps=0;
        std::vector<Entry> driver_list = em_.get_entries(); //Real time driver list from driver status
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_.is_entry_required(i->name_))
            {
              evaluate_sensor(ssc,i->available_,current_time,i->timestamp_,driver_timeout_);
            }

            if(em_.is_lidar_gps_entry_required(i->name_)==0) //Lidar1
            {
               evaluate_sensor(lidar1,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==1) //Lidar2
            {
              evaluate_sensor(lidar2,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==2) //GPS
            {
              evaluate_sensor(gps,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
        }

        //Decision making 
        if (ssc == 0)
        {
            return "s_0";
        }
        // if ssc= 1
        if((lidar1==0) && (lidar2==0) && (gps==0))
        {
            return "s_1_l1_0_l2_0_g_0";
        }
        else if((lidar1==0) && (lidar2==0) && (gps==1))
        {
            return "s_1_l1_0_l2_0_g_1";
        }
        else if((lidar1==0) && (lidar2==1) && (gps==0))
        {
            return "s_1_l1_0_l2_1_g_0";
        }
        else if((lidar1==0) && (lidar2==1) && (gps==1))
        {
            return "s_1_l1_0_l2_1_g_1";
        }
        else if((lidar1==1) && (lidar2==0) && (gps==0))
        {
            return "s_1_l1_1_l2_0_g_0";
        }
        else if((lidar1==1) && (lidar2==0) && (gps==1))
        {
            return "s_1_l1_1_l2_0_g_1";
        }
        else if((lidar1==1) && (lidar2==1) && (gps==0))
        {
            return "s_1_l1_1_l2_1_g_0";
        }
        else if((lidar1==1) && (lidar2==1) && (gps==1))
        {
            return "s_1_l1_1_l2_1_g_1";
        }
    }


    std::string DriverManager::are_critical_drivers_operational_car(long current_time)
    {
        int ssc=0;
        int lidar=0;
        int gps=0;
        std::vector<Entry> driver_list = em_.get_entries(); //Real time driver list from driver status
        for(auto i = driver_list.begin(); i < driver_list.end(); ++i)
        {
            if(em_.is_entry_required(i->name_))
            {
                evaluate_sensor(ssc,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            if(em_.is_lidar_gps_entry_required(i->name_)==0) //Lidar
            {
                evaluate_sensor(lidar,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
            else if(em_.is_lidar_gps_entry_required(i->name_)==1) //GPS
            {
                evaluate_sensor(gps,i->available_,current_time,i->timestamp_,driver_timeout_);
            }
        }

        //Decision making 
        if(ssc==1)
        {
            if((lidar==0) && (gps==0))
            {
                return "s_1_l_0_g_0";
            }
            else if((lidar==0) && (gps==1))
            {
                return "s_1_l_0_g_1";
            }
            else if((lidar==1) && (gps==0))
            {
                return "s_1_l_1_g_0";
            }
            else if((lidar==1) && (gps==1))
            {
                return "s_1_l_1_g_1";
            }
        }
        else
        {
            return "s_0";
        }

    }
    
    cav_msgs::SystemAlert DriverManager::handleSpin(bool truck,bool car,long time_now,long start_up_timestamp,long start_duration)
    {
        cav_msgs::SystemAlert alert;

        if(truck==true)
        {
            std::string status = are_critical_drivers_operational_truck(time_now);
            if(status.compare("s_1_l1_1_l2_1_g_1") == 0)
            {
                starting_up_ = false;
                alert.description = "All essential drivers are ready";
                alert.type = cav_msgs::SystemAlert::DRIVERS_READY;
                return alert;
            } 
           else if(starting_up_ && (time_now - start_up_timestamp <= start_duration))
            {
                alert.description = "System is starting up...";
                alert.type = cav_msgs::SystemAlert::NOT_READY;
                return alert;
            }
            else if((status.compare("s_1_l1_0_l2_1_g_1") == 0) || (status.compare("s_1_l1_1_l2_0_g_1") == 0))
            {
            
                alert.description = "One LIDAR Failed";
                alert.type = cav_msgs::SystemAlert::CAUTION;
                return alert;
            }
            else if((status.compare("s_1_l1_0_l2_1_g_0") == 0) || (status.compare("s_1_l1_1_l2_0_g_0") == 0))
            {   
                alert.description = "One Lidar and GPS Failed";
                alert.type = cav_msgs::SystemAlert::CAUTION;
                return alert;
            } 
            else if(status.compare("s_1_l1_1_l2_1_g_0") == 0)
            {
                alert.description = "GPS Failed";
                alert.type = cav_msgs::SystemAlert::CAUTION;
                return alert;
            }
            else if(status.compare("s_1_l1_0_l2_0_g_1") == 0)
            {
                alert.description = "Both LIDARS Failed";
                alert.type = cav_msgs::SystemAlert::WARNING;
                return alert;
            }
            else if(status.compare("s_1_l1_0_l2_0_g_0") == 0)
            {
                alert.description = "LIDARS and GPS Failed";
                alert.type = cav_msgs::SystemAlert::SHUTDOWN;
                return alert;
            }
            else if(status.compare("s_0") == 0)
            {
                alert.description = "SSC Failed";
                alert.type = cav_msgs::SystemAlert::SHUTDOWN;
                return alert;
            }
            else
            {
                alert.description = "Unknown problem assessing essential driver availability";
                alert.type = cav_msgs::SystemAlert::FATAL;
                return alert;  
            }
 
        }
        else if(car==true)
        {
            std::string status = are_critical_drivers_operational_car(time_now);
            if(status.compare("s_1_l_1_g_1") == 0)
            {
                starting_up_ = false;
                alert.description = "All essential drivers are ready";
                alert.type = cav_msgs::SystemAlert::DRIVERS_READY;
                return alert; 
            }
            else if(starting_up_ && (time_now - start_up_timestamp <= start_duration))
            {
                alert.description = "System is starting up...";
                alert.type = cav_msgs::SystemAlert::NOT_READY;
                return alert; 
            } 
            else if(status.compare("s_1_l_1_g_0") == 0)
            {
                alert.description = "GPS Failed";
                alert.type = cav_msgs::SystemAlert::CAUTION;
                return alert; 
            }
            else if(status.compare("s_1_l_0_g_1") == 0)
            {
                alert.description = "LIDAR Failed";
                alert.type = cav_msgs::SystemAlert::WARNING;
                return alert; 
            }
            else if(status.compare("s_1_l_0_g_0") == 0)
            {
                alert.description = "LIDAR, GPS Failed";
                alert.type = cav_msgs::SystemAlert::SHUTDOWN;
                return alert; 
            }
            else if(status.compare("s_0") == 0)
            {
                alert.description = "SSC Failed";
                alert.type = cav_msgs::SystemAlert::SHUTDOWN;
                return alert; 
            }
            else
            {
                alert.description = "Unknown problem assessing essential driver availability";
                alert.type = cav_msgs::SystemAlert::FATAL;
                return alert;  
            }
        }
        else
        {
            alert.description = "Need to set either truck or car flag";
            alert.type = cav_msgs::SystemAlert::FATAL;
            return alert; 
        }
    }

}

