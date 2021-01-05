
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#pragma once

#include <ros/ros.h>
#include <iostream>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/PlanType.h>
#include <mutex>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <platoon_manager.hpp>

namespace platoon_strategic
{
    struct PlatoonPlan {
        
        bool valid;
        long planStartTime;
        std::string planId;
        std::string peerId;
        PlatoonPlan():valid(""), planStartTime(0), planId(""), peerId("") {} ;
        PlatoonPlan(bool valid, long planStartTime, std::string planId, std::string peerId): 
            valid(valid), planStartTime(planStartTime), planId(planId), peerId(peerId) {}  
    };

    /**
        * A response to an MobilityRequest message.
        * ACK - indicates that the plugin accepts the MobilityRequest and will handle making any adjustments needed to avoid a conflict
        * NACK - indicates that the plugin rejects the MobilityRequest and would suggest the other vehicle replan
        * NO_RESPONSE - indicates that the plugin is indifferent but sees no conflict
    */

    enum MobilityRequestResponse {
            ACK,
            NACK,
            NO_RESPONSE
    };

    enum PlatoonState{
        STANDBY,
        LEADERWAITING,
        LEADER,
        CANDIDATEFOLLOWER,
        FOLLOWER
    };

    class PlatooningStateMachine
    {
    public:

        PlatooningStateMachine();
        
        PlatooningStateMachine(std::shared_ptr<ros::CARMANodeHandle> nh);

        /**
         * Callback method to handle mobility requests which may result in
         * state changing, trajectory re-plan and platooning info updates. 
         * @param msg the detailed proposal from other vehicles
         * @return simple yes/no response to the incoming proposal
         */
        MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg);

        /**
         * Callback method to handle mobility response.
         * @param msg response for the current plan from other vehicles
         */
        void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg);

        /**
         * Callback method to handle mobility operation.
         * @param msg the necessary operational info from other vehicles
         */
        void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg);

        cav_msgs::Maneuver composeManeuver();
        
        PlatoonState current_platoon_state;
        std::string applicantID = "";
        PlatoonPlan current_plan;

        std::string targetLeaderId = "";

        PlatoonManager pm_{nh_};


    private:
    
        std::shared_ptr<ros::CARMANodeHandle> nh_;

        ros::Publisher mob_req_pub_;

        double mvr_duration_ = 16.0;
        
        MobilityRequestResponse onMobilityRequestMessageFollower(cav_msgs::MobilityRequest &msg) const;
        void onMobilityResponseMessageFollower(cav_msgs::MobilityResponse &msg) const;
        void onMobilityOperationMessageFollower(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageLeader(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageLeader(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageLeader(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageLeaderWaiting(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessageLeaderWaiting(cav_msgs::MobilityResponse &msg) const;
        void onMobilityOperationMessageLeaderWaiting(cav_msgs::MobilityOperation &msg);

        MobilityRequestResponse onMobilityRequestMessageCandidateFollower(cav_msgs::MobilityRequest &msg) const;
        void onMobilityResponseMessageCandidateFollower(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessageCandidateFollower(cav_msgs::MobilityOperation &msg);


        MobilityRequestResponse onMobilityRequestMessageStandby(cav_msgs::MobilityRequest &msg) const;
        void onMobilityResponseMessageStandby(cav_msgs::MobilityResponse &msg) const;
        void onMobilityOperationMessageStandby(cav_msgs::MobilityOperation &msg) const;



        
        bool isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack) const;

        std::mutex plan_mutex_;

        double maxAllowedJoinTimeGap = 15.0;
        double maxAllowedJoinGap = 90;
        int maxPlatoonSize = 10;
        double vehicleLength = 5.0;
        int infoMessageInterval;
        const std::string targetPlatoonId;
        const std::string OPERATION_INFO_TYPE = "INFO";
        const std::string OPERATION_STATUS_TYPE = "STATUS";
        const std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        const std::string  MOBILITY_STRATEGY = "Carma/Platooning";
    };
}

