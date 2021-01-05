

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

#pragma once

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
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>
#include <state_machine.hpp>

// #include <leader_state.hpp>



namespace platoon_strategic
{
    class PlatoonStrategicPlugin
    {
        public:
            
            // Default constructor for PlatoonStrategicPlugin class
            PlatoonStrategicPlugin();

            // general starting point of this node
            void run();

            // local copy of pose
            boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;

            void run_states();

            PlatooningStateMachine psm_{nh_};

        protected:

            void run_leader();
            void run_leader_waiting();
            void run_candidate_follower();
            void run_follower();

        
        private:

            

            // CARMA ROS node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

            

            long waitingStartTime;
            long candidatestateStartTime;


            ros::Publisher platoon_strategic_plugin_discovery_pub_;
            ros::Publisher mob_op_pub_;
            ros::Publisher mob_req_pub_;

            ros::Subscriber pose_sub_;
            
            ros::Subscriber mob_req_sub_;
            ros::Subscriber mob_resp_sub_;
            ros::Subscriber mob_op_sub_;


            // service callbacks for carma trajectory planning
            bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            void mob_req_cb(const cav_msgs::MobilityRequest& msg);
            void mob_resp_cb(const cav_msgs::MobilityResponse& msg);
            void mob_op_cb(const cav_msgs::MobilityOperation& msg);

            // ros service servers
            ros::ServiceServer maneuver_srv_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            // trajectory frequency
            double traj_freq = 10;

            // ROS params
            double trajectory_time_length_ = 6;
            std::string control_plugin_name_ = "mpc_follower";
            

            // start vehicle speed
            double start_speed_;
            // target vehicle speed
            double target_speed_;


            int num_points = traj_freq * trajectory_time_length_;

            // initialize this node
            void initialize();

            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;



            cav_msgs::MobilityRequest mobility_req_msg_;
            cav_msgs::MobilityResponse mobility_resp_msg_;
            cav_msgs::MobilityOperation mobility_op_msg_;


            void composeMobilityOperationLeader(cav_msgs::MobilityOperation &msg, const std::string& type);
            void composeMobilityOperationFollower(cav_msgs::MobilityOperation &msg) const;
            void composeMobilityOperationLeaderWaiting(cav_msgs::MobilityOperation &msg) const;
            void composeMobilityOperationCandidateFollower(cav_msgs::MobilityOperation &msg);


            double maxAllowedJoinTimeGap = 15.0;
            double maxAllowedJoinGap = 90;
            int maxPlatoonSize = 10;
            double vehicleLength = 5.0;
            std::mutex plan_mutex_;
            int infoMessageInterval;
            long lastHeartBeatTime = 0.0;
            int statusMessageInterval = 100;
            int NEGOTIATION_TIMEOUT = 5000;  // ms
            int noLeaderUpdatesCounter = 0;
            int LEADER_TIMEOUT_COUNTER_LIMIT = 5;
            double waitingStateTimeout = 25.0; // s
            double desiredJoinGap = 30.0; // m
            double desiredJoinTimeGap = 4.0; // s


            const std::string MOBILITY_STRATEGY = "Carma/Platooning";
            const std::string OPERATION_INFO_TYPE = "INFO";
            const std::string OPERATION_STATUS_TYPE = "STATUS";
            const std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%";


            // Check these values
            std::string HostMobilityId = "hostid";
            std::string BSMID = "BSM";
            std::string MobilityId = "mobilityid";
            


    
    };
}
