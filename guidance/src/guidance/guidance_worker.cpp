/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "guidance/guidance_worker.hpp"

namespace guidance
{
    GuidanceWorker::GuidanceWorker() 
    {
        nh_ = ros::CARMANodeHandle{};
        pnh_ = ros::CARMANodeHandle{"~"};
    }

    void GuidanceWorker::system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg)
    {
        gsm->onSystemAlert(msg);
    }

    void GuidanceWorker::robot_status_cb(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        gsm->onRoboticStatus(msg);
    }

    bool GuidanceWorker::guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res)
    {
        // Translate message type from GuidanceActiveRequest to SetEnableRobotic
        ROS_INFO_STREAM("Request for guidance activation recv'd with status " << req.guidance_active);
        gsm->onSetGuidanceActive(req.guidance_active);
        cav_srvs::SetEnableRobotic srv;
        if (gsm->getCurrentState() == GuidanceStateMachine::ENGAGED) {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::ENABLE;
            res.guidance_status = true;
        } else {
            srv.request.set = cav_srvs::SetEnableRobotic::Request::DISABLE;
            res.guidance_status = false;
        }
        enable_client_.call(srv);
        return true;
    }

    bool GuidanceWorker::spin_cb()
    {
        cav_msgs::GuidanceState state;
        state.state = gsm->getCurrentState();
        state_publisher_.publish(state);
        return true;
    }

    void GuidanceWorker::create_guidance_state_machine()
    {
        gsm = guidance_state_machine_factory.createStateMachineInstance(vehicle_state_machine_type);
        if(gsm == nullptr) {
            nh_.handleException(std::invalid_argument("vehicle_state_machine_type not set correctly"));
        }
    }

    int GuidanceWorker::run()
    {
        ROS_INFO("Initalizing guidance node...");
        ros::CARMANodeHandle::setSystemAlertCallback(std::bind(&GuidanceWorker::system_alert_cb, this, std::placeholders::_1));
        // Init our ROS objects
        guidance_activate_service_server_ = nh_.advertiseService("set_guidance_active", &GuidanceWorker::guidance_acivation_cb, this);
        guidance_activated_.store(false);
        state_publisher_ = nh_.advertise<cav_msgs::GuidanceState>("state", 5);
        robot_status_subscriber_ = nh_.subscribe<cav_msgs::RobotEnabled>("robot_status", 5, &GuidanceWorker::robot_status_cb, this);
        enable_client_ = nh_.serviceClient<cav_srvs::SetEnableRobotic>("controller/enable_robotic");

        // Load the spin rate param to determine how fast to process messages
        // Default rate 10.0 Hz
        double spin_rate = pnh_.param<double>("spin_rate_hz", 10.0);

        nh_.getParam("vehicle_state_machine_type", vehicle_state_machine_type);
        create_guidance_state_machine();

        // Spin until system shutdown
        ROS_INFO_STREAM("Guidance node initialized, spinning at " << spin_rate << "hz...");
        ros::CARMANodeHandle::setSpinCallback(std::bind(&GuidanceWorker::spin_cb, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate);
        ros::CARMANodeHandle::spin();
        return 0;
    } 
}
