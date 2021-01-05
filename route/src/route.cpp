/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include "route.h"

namespace route {

    Route::Route() : tf_listener_(tf_buffer_), rg_worker_(tf_buffer_) {}

    void Route::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        // init publishers
        route_pub_ = nh_->advertise<cav_msgs::Route>("route", 1, true);
        route_event_pub_ = nh_->advertise<cav_msgs::RouteEvent>("route_event", 1);
        route_state_pub_ = nh_->advertise<cav_msgs::RouteState>("route_state", 1, true);
        route_marker_pub_= nh_->advertise<visualization_msgs::MarkerArray>("route_marker", 1, true);
        // init subscribers
        pose_sub_ = nh_->subscribe("current_pose", 1, &RouteGeneratorWorker::pose_cb, &rg_worker_);
        // init service server
        get_available_route_srv_ = nh_->advertiseService("get_available_routes", &RouteGeneratorWorker::get_available_route_cb, &rg_worker_);
        set_active_route_srv_ = nh_->advertiseService("set_active_route", &RouteGeneratorWorker::set_active_route_cb, &rg_worker_);
        abort_active_route_srv_ = nh_->advertiseService("abort_active_route", &RouteGeneratorWorker::abort_active_route_cb, &rg_worker_);
        // set world model point form wm listener
        wm_ = wml_.getWorldModel();
        rg_worker_.setWorldModelPtr(wm_);
        // load params and pass to route generator worker
        double ct_error, dt_range;
        pnh_->getParam("max_crosstrack_error", ct_error);
        pnh_->getParam("destination_downtrack_range", dt_range);
        rg_worker_.set_ctdt_param(ct_error, dt_range);
        std::string route_file_location;
        pnh_->getParam("route_file_path", route_file_location);
        rg_worker_.set_route_file_path(route_file_location);
        rg_worker_.set_publishers(route_event_pub_, route_state_pub_, route_pub_,route_marker_pub_);
    }

    void Route::run()
    {
        initialize();
        // spin with spin_callback function from RouteGeneratorWorker
        ros::CARMANodeHandle::setSpinCallback(std::bind(&RouteGeneratorWorker::spin_callback, &rg_worker_));
        ros::CARMANodeHandle::spin();
    }

}



