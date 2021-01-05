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

#include "health_monitor.h"
#include <sstream>
#include "carma_utils/CARMANodeHandle.h"


namespace health_monitor
{

    HealthMonitor::HealthMonitor()
    {
        car_ = false;
        truck_ = false;

    }
    void HealthMonitor::initialize()
    {
        // init node handles
        ROS_INFO_STREAM("Initalizing health_monitor node...");
        pnh_.reset(new ros::CARMANodeHandle("~"));
        nh_.reset(new ros::CARMANodeHandle());

        // init ros service servers, publishers and subscribers
        registered_plugin_service_server_ = nh_->advertiseService("plugins/get_registered_plugins", &HealthMonitor::registered_plugin_cb, this);
        active_plugin_service_server_ = nh_->advertiseService("plugins/get_active_plugins", &HealthMonitor::active_plugin_cb, this);
        activate_plugin_service_server_ = nh_->advertiseService("plugins/activate_plugin", &HealthMonitor::activate_plugin_cb, this);
        get_strategic_plugin_by_capability_server_ = nh_->advertiseService("plugins/get_strategic_plugin_by_capability", &PluginManager::get_strategic_plugins_by_capability, &plugin_manager_);
        get_tactical_plugin_by_capability_server_ = nh_->advertiseService("plugins/get_tactical_plugin_by_capability", &PluginManager::get_tactical_plugins_by_capability, &plugin_manager_);
        plugin_discovery_subscriber_ = nh_->subscribe<cav_msgs::Plugin>("plugin_discovery", 5, &HealthMonitor::plugin_discovery_cb, this);
        driver_discovery_subscriber_ = nh_->subscribe<cav_msgs::DriverStatus>("driver_discovery", 5, &HealthMonitor::driver_discovery_cb, this);

        // load params
        spin_rate_ = pnh_->param<double>("spin_rate_hz", 10.0);
        driver_timeout_ = pnh_->param<double>("required_driver_timeout", 500);
        startup_duration_ = pnh_->param<double>("startup_duration", 25);
        plugin_service_prefix_ = pnh_->param<std::string>("plugin_service_prefix", "");
        strategic_plugin_service_suffix_ = pnh_->param<std::string>("strategic_plugin_service_suffix", "");
        tactical_plugin_service_suffix_ = pnh_->param<std::string>("tactical_plugin_service_suffix", "");
        
        pnh_->getParam("required_plugins", required_plugins_);
        pnh_->getParam("required_drivers", required_drivers_);
        pnh_->getParam("lidar_gps_drivers", lidar_gps_drivers_); 

        truck_=false;
        car_=false;
        pnh_->getParam("truck", truck_);
        pnh_->getParam("car", car_);

        // Log parameters
        ROS_INFO_STREAM("Health Monitor Parameters {");
        ROS_INFO_STREAM("spin_rate_hz: " << spin_rate_);
        ROS_INFO_STREAM("required_driver_timeout: " << driver_timeout_);
        ROS_INFO_STREAM("startup_duration: " << startup_duration_);
        ROS_INFO_STREAM("plugin_service_prefix: " << plugin_service_prefix_);
        ROS_INFO_STREAM("strategic_plugin_service_suffix: " << strategic_plugin_service_suffix_);
        ROS_INFO_STREAM("tactical_plugin_service_suffix: " << tactical_plugin_service_suffix_);
        ROS_INFO_STREAM("truck: " << truck_);
        ROS_INFO_STREAM("car: " << car_);
        ROS_INFO_STREAM("required_plugins: [");
        for(auto p : required_plugins_) {
            ROS_INFO_STREAM("   " << p);
        }
        ROS_INFO_STREAM("  ]");

        ROS_INFO_STREAM("required_drivers: [");
        for(auto p : required_drivers_) {
            ROS_INFO_STREAM("   " << p);
        }
        ROS_INFO_STREAM("  ]");

        ROS_INFO_STREAM("lidar_gps_drivers: [");
        for(auto p : lidar_gps_drivers_) {
            ROS_INFO_STREAM("   " << p);
        }
        ROS_INFO_STREAM("  ]");
        ROS_INFO_STREAM("}");
        
         

        // initialize worker class
        plugin_manager_ = PluginManager(required_plugins_, plugin_service_prefix_, strategic_plugin_service_suffix_, tactical_plugin_service_suffix_);
        driver_manager_ = DriverManager(required_drivers_, driver_timeout_,lidar_gps_drivers_); 

        // record starup time
        start_up_timestamp_ = ros::Time::now().toNSec() / 1e6;

        ros::CARMANodeHandle::setSystemAlertCallback([&](const cav_msgs::SystemAlertConstPtr& msg) -> void {

            if (msg->type == cav_msgs::SystemAlert::FATAL)
            { 
                std::string header = "health_monitor requesting shutdown due to: " + msg->description;

                cav_msgs::SystemAlert new_msg;
                new_msg.description = header;
                new_msg.type = cav_msgs::SystemAlert::SHUTDOWN;

                nh_->publishSystemAlert(new_msg);
            }

        });
        

        ros::CARMANodeHandle::setExceptionCallback([&](const std::exception& exp) -> void {

            cav_msgs::SystemAlert new_msg;
            new_msg.type = cav_msgs::SystemAlert::SHUTDOWN;
            new_msg.description = exp.what();

            nh_->publishSystemAlert(new_msg);

        });

    }
    
    void HealthMonitor::run()
    {
        initialize();
        ros::CARMANodeHandle::setSpinCallback(std::bind(&HealthMonitor::spin_cb, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate_);
        ros::CARMANodeHandle::spin();
    }

    bool HealthMonitor::registered_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        plugin_manager_.get_registered_plugins(res);
        return true;
    }

    bool HealthMonitor::active_plugin_cb(cav_srvs::PluginListRequest& req, cav_srvs::PluginListResponse& res)
    {
        plugin_manager_.get_active_plugins(res);
        return true;
    }
    
    bool HealthMonitor::activate_plugin_cb(cav_srvs::PluginActivationRequest& req, cav_srvs::PluginActivationResponse& res)
    {
        bool answer = plugin_manager_.activate_plugin(req.pluginName, req.activated);
        if(answer) {
            res.newState = req.activated;
        }
        return answer;
    }

    void HealthMonitor::plugin_discovery_cb(const cav_msgs::PluginConstPtr& msg)
    {
        plugin_manager_.update_plugin_status(msg);
    }

    void HealthMonitor::driver_discovery_cb(const cav_msgs::DriverStatusConstPtr& msg)
    {
        // convert ros nanosecond to millisecond by the factor of 1/1e6
        driver_manager_.update_driver_status(msg, ros::Time::now().toNSec() / 1e6);
    }

    bool HealthMonitor::spin_cb()
    {
        long time_now=(ros::Time::now().toNSec() / 1e6);
        ros::Duration sd(startup_duration_);
        long start_duration=sd.toNSec() / 1e6;

        auto dm = driver_manager_.handleSpin(truck_,car_,time_now,start_up_timestamp_,start_duration);
        if (!prev_alert) {
            prev_alert = dm;
            nh_->publishSystemAlert(dm);
        } else if (prev_alert->type == dm.type && prev_alert->description.compare(dm.description) == 0) { // Do not publish duplicate alerts
            ROS_DEBUG_STREAM("No change to alert status");
        } else {
            prev_alert = dm;
            nh_->publishSystemAlert(dm);
        }
    
        return true;
    }


    void HealthMonitor::setDriverManager(DriverManager dm)
    {
        driver_manager_ = dm;
    }

    void HealthMonitor::setCarTrue()
    {
        car_ = true;
        if(truck_ == true)
            throw std::invalid_argument("truck_ = true");
    }

    void HealthMonitor::setTruckTrue()
    {
        truck_ = true;
        if(car_ == true)
            throw std::invalid_argument("car_ = true");
        
    }


}
