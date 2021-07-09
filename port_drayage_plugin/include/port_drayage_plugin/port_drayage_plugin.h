#pragma once

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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <memory>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_msgs/MobilityOperation.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/Geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <lanelet2_extension/regulatory_elements/StopRule.h>
#include <novatel_gps_msgs/Inspva.h>

namespace port_drayage_plugin
{
    /**
     * Primary Port Drayage Plugin implementation class. Split into this class
     * primarily concerned with the handling of ROS message processing and the
     * PortDrayageWorker class responsible for handling the core business logic
     * of the Port Drayage functionality.
     */ 
    class PortDrayagePlugin
    {
        private:
            std::shared_ptr<ros::CARMANodeHandle> _nh = nullptr;
            std::shared_ptr<ros::CARMANodeHandle> _pnh = nullptr;
            std::shared_ptr<ros::Subscriber> _maneuver_plan_subscriber = nullptr;
            std::shared_ptr<ros::Subscriber> _pose_subscriber = nullptr;
            std::shared_ptr<ros::Subscriber> _cur_speed_subscriber = nullptr;
            std::shared_ptr<ros::Subscriber> _inbound_mobility_operation_subscriber = nullptr;
            std::shared_ptr<ros::Subscriber> _gps_position_subscriber = nullptr;
            std::shared_ptr<ros::Publisher> _outbound_mobility_operations_publisher = nullptr;
            
            // ROS service servers
            ros::ServiceServer plan_maneuver_srv_;  

        public:
            double declaration;
            std::shared_ptr<geometry_msgs::PoseStamped> curr_pose_ = nullptr;
            geometry_msgs::Twist _cur_speed;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            /**
             * \brief Basic constructor for initializing the Port Drayage Plugin
             * 
             * \param nh A shared ptr to a public node handle for this node
             * \param pnh A shared ptr to a private node handle for this node
             */
            PortDrayagePlugin(std::shared_ptr<ros::CARMANodeHandle> nh, 
                std::shared_ptr<ros::CARMANodeHandle> pnh) :
                _nh(nh),
                _pnh(pnh) {};

            /**
             * \brief Testing constructor for initializing the Port Drayage Plugin
             * 
             * Intended for use without ROS, so the nh and pnh are left as null
             */
            PortDrayagePlugin() :
                _nh(nullptr),
                _pnh(nullptr) {};

            /**
             * \brief Begin execution of the Port Drayage Plugin functionality
             * 
             * \return The exit code of the application
             */
            int run();

            /**
             * \brief Service callback for arbitrator maneuver planning
             * \param req Plan maneuver request
             * \param resp Plan maneuver response with a list of maneuver plan
             * \return If service call successed
             */
            bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

            /**
             * \brief compose Maneuver Message to send to tactical plugin.
            * \param current_dist Start downtrack distance of the current maneuver
            * \param end_dist End downtrack distance of the current maneuver
            * \param current_speed Start speed of the current maneuver
            * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
            * \param lane_id Lanelet ID of the current maneuver
            * \param current_time Start time of the current maneuver
            * \return A stop wait maneuver message which is ready to be published
            */           
            cav_msgs::Maneuver compose_stop_and_wait_maneuver_message(double current_dist, 
                                                      double end_dist, 
                                                      double current_speed, 
                                                      double target_speed, 
                                                      int lane_id, 
                                                      ros::Time time,
                                                      double time_to_stop);

            /**
             * \brief compose Maneuver Message to send to tactical plugin.
            * \param current_dist Start downtrack distance of the current maneuver
            * \param end_dist End downtrack distance of the current maneuver
            * \param current_speed Start speed of the current maneuver
            * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
            * \param lane_id Lanelet ID of the current maneuver
            * \param current_time Start time of the current maneuver
            * \return A stop wait maneuver message which is ready to be published
            */
            cav_msgs::Maneuver compose_lane_following_maneuver_message(double current_dist, 
                                                                                        double end_dist, 
                                                                                        double current_speed, 
                                                                                        double target_speed, 
                                                                                        int lane_id, 
                                                                                        ros::Time current_time);


    };

    double estimate_distance_to_stop(double v, double a);
    double estimate_time_to_stop(double d, double v);

} // namespace port_drayage_plugin
