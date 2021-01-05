#pragma once

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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <carma_utils/CARMAUtils.h>
#include <visualization_msgs/MarkerArray.h>

#include "route_generator_worker.h"

namespace route {

    /**
     * The route package provides the following functionality:
     * - Route generation which provides the list of available routes and provides vehicle travel route description and management.
     * - Route state management which provides the current state of the route following,
     *   including tracking vehicle cross track and down track distances along the active route
     */
    class Route
    {

    public:

        /**
         * \brief Default constructor for arbitrator class
         */
        Route();

        /**
         * \brief General starting point to run this node
         */
        void run();

    private:

        // public and private node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // wm listener and pointer to the actual wm object
        carma_wm::WMListener wml_;
        carma_wm::WorldModelConstPtr wm_;

        // tf buffer holds the tree of transforms
        tf2_ros::Buffer tf_buffer_;
        // tf2 listener subscribes to the /tf and /tf_static topics
        tf2_ros::TransformListener tf_listener_;

        // route generator worker
        RouteGeneratorWorker rg_worker_;

        // publishers for route file, current route state and route event
        ros::Publisher route_pub_;
        ros::Publisher route_state_pub_;
        ros::Publisher route_event_pub_;
        ros::Publisher route_marker_pub_;

        // subscriber to current pose in the map
        ros::Subscriber pose_sub_;
        
        // route service servers
        ros::ServiceServer get_available_route_srv_;
        ros::ServiceServer set_active_route_srv_;
        ros::ServiceServer abort_active_route_srv_;

        // initialize this node before running
        void initialize();

    };
}


