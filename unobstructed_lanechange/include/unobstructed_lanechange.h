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
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/Geometry.h>
#include <basic_autonomy/basic_autonomy.h>



namespace unobstructed_lanechange
{
    /**
     * \brief Convenience class for pairing 2d points with speeds
    */ 
    struct PointSpeedPair
    {
    lanelet::BasicPoint2d point;
    double speed = 0;
    };

    class UnobstructedLaneChangePlugin
    {
        public:
            /**
             * \brief General entry point to begin the operation of this class
            */
            void run();

            /**
             * \brief Service callback for trajectory planning
             * 
             * \param req The service request
             * \param resp The service response
             * 
             * \return True if success. False otherwise
             */ 
            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

            /**
            * \brief verify if the input yield trajectory plan is valid
            * 
            * \param yield_plan input yield trajectory plan
            *
            * \return true or falss
            */
            bool validate_yield_plan(const cav_msgs::TrajectoryPlan& yield_plan, const std::string& original_plan_id);

            //Internal Variables used in unit tests
            // Current vehicle forward speed
            double current_speed_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            private:

            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

            ros::Publisher unobstructed_lanechange_plugin_discovery_pub_;

            // ros service servers
            ros::ServiceServer trajectory_srv_;
            ros::ServiceServer maneuver_srv_;

            // ros service client
            ros::ServiceClient yield_client_;

            // ROS publishers and subscribers
            cav_msgs::Plugin plugin_discovery_msg_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            ros::Timer discovery_pub_timer_;

            // trajectory frequency
            double traj_freq = 10;

            //fraction of the maneuver completed
            double maneuver_fraction_completed_ = 0;

            // ROS params
            double trajectory_time_length_ = 6;
            std::string control_plugin_name_ = "mpc_follower";
            double minimum_speed_ = 2.0;
            double max_accel_ = 1.5;
            double minimum_lookahead_distance_ = 5.0;
            double maximum_lookahead_distance_ = 25.0;
            double minimum_lookahead_speed_ = 2.8;
            double maximum_lookahead_speed_ =13.9;
            double lateral_accel_limit_ = 1.5;
            int speed_moving_average_window_size_ = 5;
            int curvature_moving_average_window_size_ = 9;
            double curvature_calc_lookahead_count_ = 1;
            int downsample_ratio_ =8;
            bool enable_object_avoidance_lc_ = false;
            double min_timestep_ = 0.1;
            int turn_downsample_ratio_ = 0.0;
            double curve_resample_step_size_ = 0.0;
            double back_distance_ = 0.0;
            double buffer_ending_downtrack_ = 0.0;

            cav_msgs::VehicleState ending_state_before_buffer_;
            
            // Time duration to ensure plan is recent
            double acceptable_time_difference_ = 1.0;
            ros::Duration time_dur_ = ros::Duration(acceptable_time_difference_);

            int num_points = traj_freq * trajectory_time_length_;


            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;
            
            // initialize this node
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
            void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);


    
    };
}