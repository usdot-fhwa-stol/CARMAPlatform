/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "state_machine.hpp"

namespace platoon_strategic
{

    PlatooningStateMachine::PlatooningStateMachine(): current_platoon_state(PlatoonState::STANDBY){};



    PlatooningStateMachine::PlatooningStateMachine(std::shared_ptr<ros::CARMANodeHandle> nh): 
        nh_(nh),
        current_platoon_state(PlatoonState::STANDBY) {
            mob_req_pub_ = nh_->advertise<cav_msgs::MobilityRequest>("mobility_request_message", 5);
        };

    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessage(cav_msgs::MobilityRequest &msg){
        switch (current_platoon_state)
        {
        case PlatoonState::FOLLOWER:
            return onMobilityRequestMessageFollower(msg);
            break;
        case PlatoonState::LEADER:
            return onMobilityRequestMessageLeader(msg);
            break;

        case PlatoonState::LEADERWAITING:
            return onMobilityRequestMessageLeaderWaiting(msg);
            break;

        case PlatoonState::CANDIDATEFOLLOWER:
            return onMobilityRequestMessageCandidateFollower(msg);
            break;
        
        case PlatoonState::STANDBY:
            return onMobilityRequestMessageStandby(msg);
            break;
        
        default:
            throw std::invalid_argument("State machine attempting to process an illegal state value");
        }
        
    }

    void PlatooningStateMachine::onMobilityResponseMessage(cav_msgs::MobilityResponse &msg){
        switch (current_platoon_state)
        {
        case PlatoonState::FOLLOWER:
            onMobilityResponseMessageFollower(msg);
            break;
        case PlatoonState::LEADER:
            onMobilityResponseMessageLeader(msg);
            break;

        case PlatoonState::LEADERWAITING:
            onMobilityResponseMessageLeaderWaiting(msg);
            break;

        case PlatoonState::CANDIDATEFOLLOWER:
            onMobilityResponseMessageCandidateFollower(msg);
            break;
        
        case PlatoonState::STANDBY:
            onMobilityResponseMessageStandby(msg);
            break;
        
        default:
            throw std::invalid_argument("State machine attempting to process an illegal state value");
        }
    }


    void PlatooningStateMachine::onMobilityOperationMessage(cav_msgs::MobilityOperation &msg){
        switch (current_platoon_state)
        {
        case PlatoonState::FOLLOWER:
            onMobilityOperationMessageFollower(msg);
            break;
        case PlatoonState::LEADER:
            onMobilityOperationMessageLeader(msg);
            break;

        case PlatoonState::LEADERWAITING:
            onMobilityOperationMessageLeaderWaiting(msg);
            break;

        case PlatoonState::CANDIDATEFOLLOWER:
            onMobilityOperationMessageCandidateFollower(msg);
            break;
        
        case PlatoonState::STANDBY:
            onMobilityOperationMessageStandby(msg);
            break;
        
        default:
            throw std::invalid_argument("State machine attempting to process an illegal state value");
        }
        
    }

    cav_msgs::Maneuver PlatooningStateMachine::composeManeuver(){
        cav_msgs::Maneuver maneuver_msg;
        

        if (current_platoon_state == PlatoonState::FOLLOWER){
            ros::Time current_time = ros::Time::now();
            maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
            double start_dist = pm_.getCurrentDowntrackDistance();
            maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::PLATOONING;
            maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
            maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
            maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicPlugin";
            maneuver_msg.lane_following_maneuver.start_dist = start_dist;
            maneuver_msg.lane_following_maneuver.start_speed = pm_.command_speed_;
            maneuver_msg.lane_following_maneuver.start_time = current_time;
            maneuver_msg.lane_following_maneuver.end_dist = start_dist + (mvr_duration_*pm_.command_speed_);
            maneuver_msg.lane_following_maneuver.end_speed = pm_.command_speed_;
            // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(mvr_duration_);
	    //TODO: Add actual lane id here
            maneuver_msg.lane_following_maneuver.lane_id = "";
        }
        
        return maneuver_msg;
    }




    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessageFollower(cav_msgs::MobilityRequest &msg) const
    {
        return MobilityRequestResponse::NO_RESPONSE;
    }


    void PlatooningStateMachine::onMobilityResponseMessageFollower(cav_msgs::MobilityResponse &msg) const
    {

    }

    void PlatooningStateMachine::onMobilityOperationMessageFollower(cav_msgs::MobilityOperation &msg)
    {
        std::string strategyParams = msg.strategy_params;
        // In the current state, we care about the STATUS message
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);
        bool isPlatoonInfoMsg = (strategyParams.find(OPERATION_INFO_TYPE) == 0);
        // If it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);
            ROS_DEBUG("Receive operation message from vehicle: " , vehicleID);
            std::string SenderBsmId = msg.header.sender_bsm_id;
            pm_.memberUpdates(vehicleID, platoonID, SenderBsmId, statusParams);
        } else if(isPlatoonInfoMsg) {
                if(msg.header.sender_id == pm_.leaderID) {
                    std::string infoParams = strategyParams.substr(OPERATION_INFO_TYPE.size()+1);

                    std::vector<std::string> parsed_params;
                    boost::algorithm::split(parsed_params, infoParams, boost::is_any_of(","));
                    std::vector<std::string> parsed_size;
                    boost::algorithm::split(parsed_size, parsed_params[3], boost::is_any_of(":"));

                    pm_.platoonSize = std::stoi(parsed_size[1]);

                    ROS_DEBUG("Update from the lead: the current platoon size is " , pm_.getTotalPlatooningSize());
                }
        }else{
            ROS_DEBUG("Ignore other operation messages with params: " , strategyParams);
        }
    }


    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessageLeader(cav_msgs::MobilityRequest &msg)
    {
        // If we received a JOIN request, we decide whether we allow that CAV to join.
        // If we agree on its join, we need to change to LeaderWaiting state.
        if (msg.plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR){
            // We are currently checking two basic JOIN conditions:
            //     1. The size limitation on current platoon based on the plugin's parameters.
            //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
            std::string params = msg.strategy_params;
            std::string applicantId = msg.header.sender_id;
            ROS_DEBUG("Receive mobility JOIN request from " , applicantId, " and PlanId = " , msg.header.plan_id);
            ROS_DEBUG("The strategy parameters are " , params);
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,SPEED:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            
            std::vector<std::string> parsed_params;
            boost::algorithm::split(parsed_params, params, boost::is_any_of(","));

            std::vector<std::string> parsed_size;
            boost::algorithm::split(parsed_size, parsed_params[0], boost::is_any_of(":"));
            int applicantSize = std::stoi(parsed_size[1]);

            std::vector<std::string> parsed_speed;
            boost::algorithm::split(parsed_speed, parsed_params[1], boost::is_any_of(":"));
            double applicantCurrentSpeed = std::stod(parsed_size[1]);

            std::vector<std::string> parsed_dtd;
            boost::algorithm::split(parsed_dtd, parsed_params[1], boost::is_any_of(":"));
            double applicantCurrentDtd = std::stod(parsed_size[1]);

            // Check if we have enough room for that applicant
            int currentPlatoonSize = pm_.getTotalPlatooningSize();
            bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= maxPlatoonSize;
            if(hasEnoughRoomInPlatoon) {
                ROS_DEBUG("The current platoon has enough room for the applicant with size " , applicantSize);
                double currentRearDtd = pm_.getPlatoonRearDowntrackDistance();
                ROS_DEBUG("The current platoon rear dtd is " , currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG("The gap between current platoon rear and applicant is " , currentGap , "m or " , currentTimeGap , "s");
                if(currentGap < 0) {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= maxAllowedJoinGap) || (currentTimeGap <= maxAllowedJoinTimeGap);
                if(isDistanceCloseEnough) {
                    ROS_DEBUG("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG("Change to LeaderWaitingState and waiting for " , msg.header.sender_id , " to join");
                    current_platoon_state = PlatoonState::LEADERWAITING;
                    // applicantID = applicantId;
                    return MobilityRequestResponse::ACK;
                }
                else {
                    ROS_DEBUG("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else {
                ROS_DEBUG("The current platoon does not have enough room for applicant of size " , applicantSize , ". NACK");
                return MobilityRequestResponse::NACK;
            }
        }
        else {
            ROS_DEBUG("Received mobility request with type " , msg.plan_type.type , " and ignored." );
        }
        return MobilityRequestResponse::NO_RESPONSE;
    }


    void PlatooningStateMachine::onMobilityResponseMessageLeader(cav_msgs::MobilityResponse &msg)
    {
        // Only care the response message for the plan for which we are waiting
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            if (current_plan.valid){
                if ((current_plan.planId == msg.header.plan_id) && (current_plan.peerId == msg.header.sender_id)){
                    if (msg.is_accepted){
                        ROS_DEBUG("Received positive response for plan id = " , current_plan.planId);
                        ROS_DEBUG("Change to CandidateFollower state and notify trajectory failure in order to replan");
        //                 // Change to candidate follower state and request a new plan to catch up with the front platoon
                        current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
                        targetLeaderId = current_plan.peerId;
                        
                    } else{
                        ROS_DEBUG("Received negative response for plan id = " , current_plan.planId);
                        // Forget about the previous plan totally
                            current_plan.valid = false;
                    }
                } else{
                    ROS_DEBUG("Ignore the response message because planID match: " , current_plan.planId == msg.header.plan_id);
                    ROS_DEBUG("My plan id = " , current_plan.planId , " and response plan Id = " , msg.header.plan_id);
                    ROS_DEBUG("And peer id match: " , current_plan.peerId == msg.header.sender_id);
                    ROS_DEBUG("Expected peer id = " , current_plan.peerId , " and response sender Id = " , msg.header.sender_id);
                }
            }
        }

    }

    void PlatooningStateMachine::onMobilityOperationMessageLeader(cav_msgs::MobilityOperation &msg)
    {
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id;
        std::string platoonId = msg.header.plan_id;
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in our current platoon

        bool isPlatoonInfoMsg = (strategyParams.find(OPERATION_INFO_TYPE) == 0);// = strategyParams.startsWith(PlatooningPlugin.OPERATION_INFO_TYPE);
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);;// = strategyParams.startsWith(PlatooningPlugin.OPERATION_STATUS_TYPE);
        {
            std::lock_guard<std::mutex> lock(plan_mutex_);

            bool isNotInNegotiation = (!current_plan.valid);
            if(isPlatoonInfoMsg && isNotInNegotiation) {
                // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d
                // TODO In future, we should remove downtrack distance from this string and send XYZ location in ECEF
                std::vector<std::string> parsed_strategy_params;
                boost::algorithm::split(parsed_strategy_params, strategyParams, boost::is_any_of(","));

                std::vector<std::string> parsed_BsmId;
                boost::algorithm::split(parsed_BsmId, parsed_strategy_params[0], boost::is_any_of(":"));
                std::string rearVehicleBsmId = parsed_BsmId[1];

                std::vector<std::string> parsed_dtd;
                boost::algorithm::split(parsed_dtd, parsed_strategy_params[4], boost::is_any_of(":"));
                double rearVehicleDtd = std::stod(parsed_dtd[1]);
                // We are trying to validate is the platoon rear is right in front of the host vehicle
                if(isVehicleRightInFront(rearVehicleBsmId, rearVehicleDtd)) {
                    ROS_DEBUG("Found a platoon with id = " , platoonId , " in front of us.");
                    cav_msgs::MobilityRequest request;
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toSec();
                    request.header.plan_id = planId;
                    request.header.recipient_id = senderId;
                    cav_msgs::LocationECEF location; //from GPS?
                    request.location = location;
                    request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
                    request.strategy = MOBILITY_STRATEGY;


                    double total_platoon_size = pm_.getTotalPlatooningSize();
                    double current_speed = pm_.current_speed_;
                    double current_downtrack = pm_.getCurrentDowntrackDistance();

                    boost::format fmter(JOIN_AT_REAR_PARAMS);
                    fmter %total_platoon_size;
                    fmter %current_speed;
                    fmter %current_downtrack;
 
                    std::string strategyParamsString = fmter.str(); 
                    request.strategy_params = strategyParamsString;
                    request.urgency = 50;
                    mob_req_pub_.publish(request);
                    PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, senderId);
                    current_plan = *new_plan;

                    
                    ROS_DEBUG("Publishing request to leader " , senderId , " with params " , request.strategy_params , " and plan id = " , request.header.plan_id);
                    // this.potentialNewPlatoonId = platoonId;
                } else{
                        ROS_DEBUG("Ignore platoon with platoon id: " , platoonId , " because it is not right in front of us");
                }
            }
            else if(isPlatoonStatusMsg) {
                // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
                std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);// = strategyParams.substring(PlatooningPlugin.OPERATION_STATUS_TYPE.length() + 1);
                ROS_DEBUG("Receive operation status message from vehicle: " , senderId , " with params: " , statusParams);
                std::string SenderBsmId = msg.header.sender_bsm_id;
                pm_.memberUpdates(senderId, platoonId, SenderBsmId, statusParams);
            }
            else {
                ROS_DEBUG("Receive operation message but ignore it because isPlatoonInfoMsg = " , isPlatoonInfoMsg 
                        , ", isNotInNegotiation = " , isNotInNegotiation , " and isPlatoonStatusMsg = " , isPlatoonStatusMsg);
            }

        }
    }

    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessageLeaderWaiting(cav_msgs::MobilityRequest &msg)
    {
        bool isTargetVehicle = (msg.header.sender_id == applicantID);
        bool isCandidateJoin = (msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN);
        if(isTargetVehicle && isCandidateJoin) {
            ROS_DEBUG("Target vehicle " , applicantID , " is actually joining.");
            ROS_DEBUG("Changing to PlatoonLeaderState and send ACK to target vehicle");
            current_platoon_state = PlatoonState::LEADER;
            return MobilityRequestResponse::ACK;
        }
        else{
            ROS_DEBUG("Received platoon request with vehicle id = " , msg.header.sender_id);
            ROS_DEBUG("The request type is " , msg.plan_type.type,  " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }

    void PlatooningStateMachine::onMobilityResponseMessageLeaderWaiting(cav_msgs::MobilityResponse &msg) const
    {
        // Do Nothing
    }

    void PlatooningStateMachine::onMobilityOperationMessageLeaderWaiting(cav_msgs::MobilityOperation &msg)
    {
        // We still need to handle STATUS operation message from our platoon
        std::string strategyParams = msg.strategy_params;
        std::string OPERATION_STATUS_TYPE;
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0); 
        if (isPlatoonStatusMsg){
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1); 
            std::string SenderBsmId = msg.header.sender_bsm_id;
            pm_.memberUpdates(vehicleID, platoonID, SenderBsmId, statusParams);
            ROS_DEBUG("Received platoon status message from " , msg.header.sender_id);
        }
        else
        {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params + " but ignored.");
        }
    }

    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessageCandidateFollower(cav_msgs::MobilityRequest &msg) const
    {
        // No need to response as a follower
        return MobilityRequestResponse::NO_RESPONSE;
    }
    void PlatooningStateMachine::onMobilityResponseMessageCandidateFollower(cav_msgs::MobilityResponse &msg)
    {
        // Waiting on response for the current Candidate-Join plan
        if (current_plan.valid){
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if (current_plan.valid){
                    bool isForCurrentPlan = (msg.header.plan_id == current_plan.planId);
                    bool isFromTargetVehicle = (msg.header.sender_id == targetLeaderId);
                    if(isForCurrentPlan && isFromTargetVehicle) {
                        if(msg.is_accepted) {
                        // We change to follower state and start to actually follow that leader
                        // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                        ROS_DEBUG("The leader " , msg.header.sender_id , " agreed on our join. Change to follower state.");
                        pm_.changeFromLeaderToFollower(targetPlatoonId);
                        current_platoon_state = PlatoonState::FOLLOWER;
                        // pluginServiceLocator.getArbitratorService().requestNewPlan(this.trajectoryEndLocation);
                        }
                        else{
                            // We change back to normal leader state and try to join other platoons
                            ROS_DEBUG("The leader " , msg.header.sender_id , " does not agree on our join. Change back to leader state.");
                            current_platoon_state = PlatoonState::LEADER;
                        }
                    }
                    else{
                        ROS_DEBUG("Ignore received response message because it is not for the current plan.");
                    }
                }
            }
        }
        else{
            ROS_DEBUG("Ignore received response message because we are not in any negotiation process.");
        }
    }
    void PlatooningStateMachine::onMobilityOperationMessageCandidateFollower(cav_msgs::MobilityOperation &msg)
    {
        // We still need to handle STATUS operAtion message from our platoon
        std::string strategyParams = msg.strategy_params;
        std::string OPERATION_STATUS_TYPE;
        bool isPlatoonStatusMsg = (strategyParams.find(OPERATION_STATUS_TYPE) == 0);
        if (isPlatoonStatusMsg){
            std::string vehicleId = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size()+1);
            std::string SenderBsmId = msg.header.sender_bsm_id;
            pm_.memberUpdates(vehicleId, platoonId, SenderBsmId, statusParams);
            ROS_DEBUG("Received platoon status message from " , vehicleId);
        }
        else {
            ROS_DEBUG("Received a mobility operation message with params " , msg.strategy_params , " but ignored.");
        }
    }


    MobilityRequestResponse PlatooningStateMachine::onMobilityRequestMessageStandby(cav_msgs::MobilityRequest &msg) const
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        return MobilityRequestResponse::NO_RESPONSE;
    }
    void PlatooningStateMachine::onMobilityResponseMessageStandby(cav_msgs::MobilityResponse &msg) const
    {
        // Do Nothing
    }
    void PlatooningStateMachine::onMobilityOperationMessageStandby(cav_msgs::MobilityOperation &msg) const
    {
        // Do Nothing
    }

    bool PlatooningStateMachine::isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack) const {
        double currentDtd = pm_.getCurrentDowntrackDistance();
        if(downtrack > currentDtd) {
            std::clog << "Found a platoon in front. We are able to join" << std::endl;
            return true;
        } else {
            std::clog <<"Ignoring platoon from our back." << std::endl;
            std::clog << "The front platoon dtd is " << downtrack << " and we are current at " << currentDtd << std::endl;
            return false;
        }
    }

        
}
