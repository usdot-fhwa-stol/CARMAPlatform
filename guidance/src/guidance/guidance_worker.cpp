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

#include "guidance/guidance_worker.hpp"

namespace guidance
{

    void GuidanceWorker::system_alert_cb(const cav_msgs::SystemAlertConstPtr& msg)
    {
        gsm_.onSystemAlert(msg);
    }

    void GuidanceWorker::robot_status_cb(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        gsm_.onRoboticStatus(msg);
    }

    void GuidanceWorker::route_event_cb(const cav_msgs::RouteEventConstPtr& msg)
    {
        gsm_.onRouteEvent(msg);
    }

    bool GuidanceWorker::guidance_acivation_cb(cav_srvs::SetGuidanceActiveRequest& req, cav_srvs::SetGuidanceActiveResponse& res)
    {
        // Translate message type from GuidanceActiveRequest to SetEnableRobotic
        ROS_INFO_STREAM("Request for guidance activation recv'd with status " << req.guidance_active);
        if(!req.guidance_active)
        {
            cav_srvs::SetEnableRobotic srv;
            srv.request.set = cav_srvs::SetEnableRobotic::Request::DISABLE;
            enable_client_.call(srv);
        }
        gsm_.onSetGuidanceActive(req.guidance_active);
        res.guidance_status = (gsm_.getCurrentState() == GuidanceStateMachine::ACTIVE);
        return true;
    }

    bool GuidanceWorker::spin_cb()
    {
        if(gsm_.shouldCallSetEnableRobotic()) {
            cav_srvs::SetEnableRobotic srv;
            srv.request.set = cav_srvs::SetEnableRobotic::Request::ENABLE;
            enable_client_.call(srv);
        }
        cav_msgs::GuidanceState state;
        state.state = gsm_.getCurrentState();
        state_publisher_.publish(state);
        return true;
    }

    int GuidanceWorker::run()
    {
        ROS_INFO("Initalizing guidance node...");
        ros::CARMANodeHandle::setSystemAlertCallback(std::bind(&GuidanceWorker::system_alert_cb, this, std::placeholders::_1));
        // Init our ROS objects
        guidance_activate_service_server_ = nh_.advertiseService("set_guidance_active", &GuidanceWorker::guidance_acivation_cb, this);
        state_publisher_ = nh_.advertise<cav_msgs::GuidanceState>("state", 5);
        robot_status_subscriber_ = nh_.subscribe<cav_msgs::RobotEnabled>("robot_status", 5, &GuidanceWorker::robot_status_cb, this);
        route_event_subscriber_ = nh_.subscribe<cav_msgs::RouteEvent>("route_event", 5, &GuidanceWorker::route_event_cb, this);
        enable_client_ = nh_.serviceClient<cav_srvs::SetEnableRobotic>("controller/enable_robotic");

        // Load the spin rate param to determine how fast to process messages
        // Default rate 10.0 Hz
        double spin_rate = pnh_.param<double>("spin_rate_hz", 10.0);

        // Spin until system shutdown
        ROS_INFO_STREAM("Guidance node initialized, spinning at " << spin_rate << "hz...");
        ros::CARMANodeHandle::setSpinCallback(std::bind(&GuidanceWorker::spin_cb, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate);
        ros::CARMANodeHandle::spin();

        // return
        return 0;
    } 
}
