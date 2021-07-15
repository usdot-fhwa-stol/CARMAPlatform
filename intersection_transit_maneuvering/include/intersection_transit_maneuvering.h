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
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <inlanecruising_plugin/smoothing/SplineI.h>
#include "inlanecruising_config.h"
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <basic_autonomy/helper_functions.h>
#include <inlanecruising_plugin/inlanecruising_plugin.h>

namespace intersection_transit_maneuvering
{


    class IntersectionTransitManeuvering
    {
        public:
        /**
        * \brief Constructor
        * 
       
        */ 
        IntersectionTransitManeuvering();
                       
        /**
         * \brief Service callback for trajectory planning
         * 
         * \param req The service request
         * \param resp The service response
         * 
         * \return True if success. False otherwise
         */ 
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

        /**
         * \brief General entry point to begin the operation of this class
         */
        void run();


        /**
         *  \brief Converts a sequence of INTERSECTION_TRANSIT maneuvers to LANE_FOLLOWING maneuvers
         * 
         * \param maneuvers The list of maneuvers to convert
         * 
         * \return The new list of converted maneuvers
        */
        std::vector<cav_msgs::Maneuver> convert_maneuver_plan(const std::vector<cav_msgs::Maneuver>& maneuvers);

        private:
            //CARMA ROS node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_,pnh_;

            // ROS service servers
            ros::ServiceServer trajectory_srv_;

            //ROS publishers and subscribers
            ros::Publisher plugin_discovery_pub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            ros::Publisher jerk_pub_;
            ros::Timer discovery_pub_timer_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            //Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            //Calculated jerk for maneuver in m/s3
            double jerk_ =0.0;
            //Total time required to complete the maneuver
            double maneuver_time_;

            /**
            * \brief Initialize ROS publishers, subscribers, service servers and service clients
            */
            void initialize();

            /**
            * \brief Callback for the pose subscriber, which will store latest pose locally
            * \param msg Latest pose message
            */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            /**
            * \brief Callback for the twist subscriber, which will store latest twist locally
            * \param msg Latest twist message
            */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);

  
            carma_wm::WorldModelConstPtr wm_;


    }
















}