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
#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/UpcomingLaneChangeStatus.h>

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver typees
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                    ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        throw new std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id"))))))


namespace route_following_plugin
{

    class RouteFollowingPlugin
    {

    public:

        // constant speed limit for test
        // TODO Once world_model and vector map is ready, it should be removed
        static constexpr double TWENTY_FIVE_MPH_IN_MS = 11.176;
        static constexpr double FIFTEEN_MPH_IN_MS = 6.7056;

        /**
         * \brief Default constructor for RouteFollowingPlugin class
         */
        RouteFollowingPlugin();

        /**
         * \brief General entry point to begin the operation of this class
         */
        void run();

        /**
         * \brief Given a LaneletPath object, find index of the lanelet which has target_id as its lanelet ID
         * \param target_id The laenlet ID this function is looking for
         * \param path A list of lanelet with different lanelet IDs
         * \return Index of the target lanelet in the list
         */
        int findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path);

        /**
         * \brief Compose a lane keeping maneuver message based on input params
         * \param current_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param current_speed Start speed of the current maneuver
         * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
         * \param lane_id Lanelet ID of the current maneuver
         * \param current_time Start time of the current maneuver passed as reference
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time);
        /**
         * \brief Compose a stop and wait maneuver message based on input params
         * \param current_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param current_speed Start speed of the current maneuver
         * \param start_lane_id Starting Lanelet ID of the current maneuver
         * \param target_speed Target speed of the current maneuver
         * \param current_time Start time of the current maneuver passed as reference
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeStopandWaitManeuverMessage(double current_dist, double end_dist, double current_speed, int start_lane_id, int target_lane_id, ros::Time& current_time, double end_time);
        /**
         * \brief Compose a lane change maneuver message based on input params
         * \param current_dist Start downtrack distance of the current maneuver
         * \param end_dist End downtrack distance of the current maneuver
         * \param current_speed Start speed of the current maneuver
         * \param start_lane_id Starting Lanelet ID of the current maneuver
         * \param ending_lane_id Ending Lanelet ID of the current maneuver
         * \param target_speed Target speed of the current maneuver
         * \param current_time Start time of the current maneuver passed as reference
         * \return A lane keeping maneuver message which is ready to be published
         */
        cav_msgs::Maneuver composeLaneChangeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int starting_lane_id, int ending_lane_id, ros::Time& current_time);
        /**
         * \brief Given a prior maneuver plan, update current speed, current_progress and current lanelet id for requested plan
         * \param maneuver A maneuver message from the end of the prior plan, used to update current request
         * \param speed current speed value passed as reference, updated in the function
         * \param current_progress the current_progress passed as reference, updated in the function
         * \param lane_id The lanelet id of the current lanelet passed as reference, updated in the function
         * \return Whether we need a lanechange to reach to the next lanelet in the shortest path
         */
        void updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id);
        /**
         * \brief Given a LaneletRelations and ID of the next lanelet in the shortest path
         * \param relations LaneletRelations relative to the previous lanelet
         * \param target_id ID of the next lanelet in the shortest path
         * \return Whether we need a lanechange to reach to the next lanelet in the shortest path
         */
        bool identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id);

        /**
         * \brief Service callback for arbitrator maneuver planning
         * \param req Plan maneuver request
         * \param resp Plan maneuver response with a list of maneuver plan
         * \return If service call successed
         */
        bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);
        /**
         * \brief Given a Lanelet, find it's associated Speed Limit from the vector map
         * \param llt Constant Lanelet object
         * \return value of speed limit as a double, returns default value of 25 mph
         */
        double findSpeedLimit(const lanelet::ConstLanelet& llt);

        //Internal Variables used in unit tests
        // Current vehicle forward speed
        double current_speed_;

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;

        // wm listener pointer and pointer to the actual wm object
        std::shared_ptr<carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;

        // config limit for vehicle speed limit set as ros parameter
        double config_limit=0.0;

         /**
         * \brief ComposeLaneChangeStatus() Given lane change status
         * \param lane_change_start_dist,starting_lanelet,ending_lanelet,current_downtrack
         */
        cav_msgs::UpcomingLaneChangeStatus ComposeLaneChangeStatus(double lane_change_start_dist,lanelet::ConstLanelet starting_lanelet,lanelet::ConstLanelet ending_lanelet,double current_downtrack);

        //Tactical plugin being used for planning lane change
        std::string lane_change_plugin_ = "CooperativeLaneChangePlugin";

    private:

        // CARMA ROS node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        std::shared_ptr<ros::CARMANodeHandle> pnh2_; //Global Scope

        // ROS publishers and subscribers
        ros::Publisher plugin_discovery_pub_;
        ros::Publisher upcoming_lane_change_status_pub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;
        ros::Timer discovery_pub_timer_;

        // ROS service servers
        ros::ServiceServer plan_maneuver_srv_;        

        // Minimal duration of maneuver, loaded from config file
        double mvr_duration_;
        //Jerk used to come to stop at end of route
        double jerk_ = 0.05;
        //extra time allowed for lane changing, in order to make transition smooth
        double buffer_lanechange_time_ = 1.0;

        //lane change constant
        static constexpr double LATERAL_ACCELERATION_LIMIT_IN_MS=2.00;
        static const int MAX_LANE_WIDTH=3.70;
        static constexpr double LANE_CHANGE_TIME_MAX=sqrt(2*MAX_LANE_WIDTH/LATERAL_ACCELERATION_LIMIT_IN_MS);

        //Small constant to compare double with approx zero
        const double epislon_ = 0.001;

        // Plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        //Upcoming Lane Change message
        cav_msgs::UpcomingLaneChangeStatus upcoming_lane_change_status_msg_;

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
        void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);
 
    
    
    };

}