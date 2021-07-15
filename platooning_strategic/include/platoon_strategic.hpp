

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

#pragma once

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanManeuvers.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlatooningInfo.h>
#include <cav_msgs/PlanType.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/traffic_rules/CarmaUSTrafficRules.h>
#include "platoon_config.h"
#include <platoon_manager.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>

namespace platoon_strategic
{
    using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
    using MobilityResponseCB = std::function<void(const cav_msgs::MobilityResponse&)>;
    using MobilityRequestCB = std::function<void(const cav_msgs::MobilityRequest&)>;
    using MobilityOperationCB = std::function<void(const cav_msgs::MobilityOperation&)>;
    using PlatooningInfoCB = std::function<void(const cav_msgs::PlatooningInfo&)>;

    class PlatoonStrategicPlugin
    {
        public:
            
            // Default constructor for PlatoonStrategicPlugin class
            PlatoonStrategicPlugin();

            /**
            * \brief Constructor
            * 
            * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
            * \param config The configuration to be used for this object
            * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
            */ 
            PlatoonStrategicPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
                                PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher,
                                MobilityRequestCB mobility_request_publisher, MobilityOperationCB mobility_operation_publisher,
                                PlatooningInfoCB platooning_info_publisher);


            // callback functions
            void mob_op_cb(const cav_msgs::MobilityOperation& msg);
            void mob_req_cb(const cav_msgs::MobilityRequest& msg);
            void mob_resp_cb(const cav_msgs::MobilityResponse& msg);
            MobilityRequestResponse handle_mob_req(const cav_msgs::MobilityRequest& msg);

            // service callbacks for carma trajectory planning
            bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);
            int findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path);

            cav_msgs::Maneuver composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time);
            void updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id);
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            cav_msgs::PlatooningInfo compose_platoon_info_msg();
            /**
            * \brief Callback for the twist subscriber, which will store latest twist locally
            * \param msg Latest twist message
            */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);
            void cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg);

            void bsm_cb(const cav_msgs::BSMConstPtr& msg);

            void georeference_cb(const std_msgs::StringConstPtr& msg);

            
            bool onSpin();


            PlatoonManager pm_;
            
            
            void run_leader();
            void run_leader_waiting();
            void run_candidate_follower();
            void run_follower();
            
            void lookupECEFtoMapTransform();

            cav_msgs::LocationECEF pose_ecef_point_;

        
        private:

            
            PublishPluginDiscoveryCB plugin_discovery_publisher_;
            MobilityRequestCB mobility_request_publisher_;
            MobilityResponseCB mobility_response_publisher_;
            MobilityOperationCB mobility_operation_publisher_;
            PlatooningInfoCB platooning_info_publisher_;


            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            PlatoonPluginConfig config_;
            // local copy of pose
            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;
            
            //Internal Variables used in unit tests
            // Current vehicle forward speed
            double cmd_speed_ = 0;
            double current_speed_ = 0;
            double current_downtrack_ = 0;
            double current_crosstrack_ = 0;

            long waitingStartTime = 0;
            long candidatestateStartTime = 0;

            std::string potentialNewPlatoonId = "";
            std::string targetPlatoonId = "";


            ros::Publisher platoon_strategic_plugin_discovery_pub_;
            ros::Publisher mob_op_pub_;
            ros::Publisher mob_req_pub_;

            ros::Subscriber pose_sub_;
            
            ros::Subscriber mob_req_sub_;
            ros::Subscriber mob_resp_sub_;
            ros::Subscriber mob_op_sub_;

            bool isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack);
            
            double findSpeedLimit(const lanelet::ConstLanelet& llt);
            
            MobilityRequestResponse mob_req_cb_leader(const cav_msgs::MobilityRequest& msg);
            MobilityRequestResponse mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg);
            MobilityRequestResponse mob_req_cb_follower(const cav_msgs::MobilityRequest& msg);
            MobilityRequestResponse mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg);
            MobilityRequestResponse mob_req_cb_standby(const cav_msgs::MobilityRequest& msg);
            
            void mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg);
            void mob_resp_cb_leaderwaiting(const cav_msgs::MobilityResponse& msg);
            void mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg);
            void mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg);
            void mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg);

            

            
            
            void mob_op_cb_leader(const cav_msgs::MobilityOperation& msg);
            void mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg);
            void mob_op_cb_follower(const cav_msgs::MobilityOperation& msg);
            void mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg);
            void mob_op_cb_standby(const cav_msgs::MobilityOperation& msg);

            // TF listenser
            tf2_ros::Buffer tf2_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
            geometry_msgs::TransformStamped tf_;

            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

            
            bool map_loaded_ = false;

            std::string host_bsm_id_ = "";

            // ros service servers
            ros::ServiceServer maneuver_srv_;

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;


            cav_msgs::MobilityRequest mobility_req_msg_;
            cav_msgs::MobilityResponse mobility_resp_msg_;
            cav_msgs::MobilityOperation mobility_op_msg_;


            cav_msgs::MobilityOperation composeMobilityOperationLeader(const std::string& type);
            cav_msgs::MobilityOperation composeMobilityOperationFollower();
            cav_msgs::MobilityOperation composeMobilityOperationLeaderWaiting();
            cav_msgs::MobilityOperation composeMobilityOperationCandidateFollower();

            cav_msgs::LocationECEF pose_to_ecef(geometry_msgs::PoseStamped pose_msg);
            lanelet::BasicPoint2d ecef_to_map_point(cav_msgs::LocationECEF ecef_point);

            



            double maxAllowedJoinTimeGap_ = 15.0;
            double maxAllowedJoinGap_ = 90;
            int maxPlatoonSize_ = 10;
            double vehicleLength_ = 5.0;
            std::mutex plan_mutex_;
            int infoMessageInterval_ = 200; //ms
            long lastHeartBeatTime = 0.0;
            int statusMessageInterval_ = 100;
            
            int noLeaderUpdatesCounter = 0;
            int LEADER_TIMEOUT_COUNTER_LIMIT = 5;
            double waitingStateTimeout = 25.0; // s
            double desiredJoinGap = 30.0; // m
            double desiredJoinTimeGap = 4.0; // s

            PlatoonPlan current_plan_;


            // Platooning Plugin Info 
            const std::string MOBILITY_STRATEGY = "Carma/Platooning";
            const std::string OPERATION_INFO_TYPE = "INFO";
            const std::string OPERATION_STATUS_TYPE = "STATUS";
            const std::string OPERATION_INFO_PARAMS   = "INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f,ECEFX:%.2f,ECEFY:%.2f,ECEFZ:%.2f";
            const std::string OPERATION_STATUS_PARAMS = "STATUS|CMDSPEED:%1%,DTD:%2%,SPEED:%3%,ECEFX:%4%,ECEFY:%5%,ECEFZ:%6%";
            const std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%,ECEFX:%4%,ECEFY:%5%,ECEFZ:%6%";
            int NEGOTIATION_TIMEOUT = 5000;  // ms
            


            // Check these values
            std::string HostMobilityId = "hostid";
            std::string BSMID = "BSM";

            // leader_waiting applicant id
            std::string lw_applicantId_ = "";

            std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core)
            {
                std::string res = "";
                for (size_t i=0; i<bsm_core.id.size(); i++)
                {
                res+=std::to_string(bsm_core.id[i]);
                }
                return res;
            }
            



    };
}
