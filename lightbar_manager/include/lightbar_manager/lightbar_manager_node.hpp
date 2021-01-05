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

#ifndef _LIGHTBAR_MANAGER_H
#define _LIGHTBAR_MANAGER_H

#include <string>
#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <vector>
#include <map>

#include <cav_msgs/LightBarCDAType.h>
#include <cav_msgs/LightBarIndicator.h>
#include <cav_msgs/LightBarIndicatorControllers.h>
#include <cav_msgs/LightBarStatus.h>
#include <cav_msgs/GuidanceState.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>

#include <cav_srvs/RequestIndicatorControl.h>
#include <cav_srvs/ReleaseIndicatorControl.h>
#include <cav_srvs/SetLightBarIndicator.h>
#include <cav_srvs/SetLights.h>

#include "lightbar_manager/lightbar_manager_worker.hpp"

namespace lightbar_manager
{

class LightBarManager
{
    public:

        /*!
        * \brief Default constructor for LightBarManager node
        */
        LightBarManager(const std::string& node_name);

        /*!
        * \brief Initialize ROS related functions. Pass "test" to setup ROS parameters for unit testing
        */
        void init(std::string mode = "");

        /*!
        * \brief Begin normal execution of LightBarManager worker. Will take over control flow of program and exit from here.
        * \return The exit status of this program
        */
        int run();

        /*!
        * \brief Get ptr to lightbar_manager_worker (for ease of unit testing)
        * \return LightBarManagerWorker
        */
        std::shared_ptr<LightBarManagerWorker> getWorker();
 
        /*!
        * \brief Miscellaneous function that forces the state to disengaged and turn off all indicators.
        * Used in special demo cases as well as when carma is disengaged
        */
        void turnOffAll();

        /*!
        * \brief Try to turn the given indicator ON or OFF (comm with hardware) upon the given component's request
        * \return Returns the status code whether if the request was successful or not
        */
        int setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, const std::string& requester_name);

        /*!
        * \brief Callback function for turning signal
        * \return 
        */
        void turnSignalCallback(const automotive_platform_msgs::TurnSignalCommandPtr& msg_ptr);

    private:
        /*!
        * \brief Helper function that sets up ROS parameters for unit test
        * \return none
        */
        void setupUnitTest();

        // Node Data
        std::string node_name_;
        double spin_rate_;
        std::map<lightbar_manager::LightBarIndicator, std::__cxx11::string> prev_owners_before_turn_;

        // spin callback function
        bool spinCallBack();

        // Message/service callbacks
        bool requestControlCallBack(cav_srvs::RequestIndicatorControlRequest& req, cav_srvs::RequestIndicatorControlResponse& res);
        bool releaseControlCallBack(cav_srvs::ReleaseIndicatorControlRequest& req, cav_srvs::ReleaseIndicatorControlResponse& res);
        bool setIndicatorCallBack(cav_srvs::SetLightBarIndicatorRequest& req, cav_srvs::SetLightBarIndicatorResponse& res);
        void stateChangeCallBack(const cav_msgs::GuidanceStateConstPtr& msg_ptr);

        // Service servers/clients
        ros::ServiceServer request_control_server_;
        ros::ServiceServer release_control_server_;
        ros::ServiceServer set_indicator_server_;
        ros::ServiceClient lightbar_driver_client_;

        // Publishers
        ros::Publisher indicator_control_publisher_;

        // Subscribers
        ros::Subscriber guidance_state_subscriber_;
        ros::Subscriber turn_signal_subscriber_;

        // Node handles
        ros::CARMANodeHandle nh_{"lightbar_manager"}, pnh_{"~"};

        // LightBarManager Worker
        std::shared_ptr<LightBarManagerWorker> lbm_;

}; //class LightBarManagerNode
} // namespace lightbar_manager

#endif //_LIGHTBAR_MANAGER_H