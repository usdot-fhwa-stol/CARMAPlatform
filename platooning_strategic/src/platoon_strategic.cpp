
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
#include <ros/ros.h>
#include <string>
#include "platoon_strategic.hpp"

namespace platoon_strategic
{
    PlatoonStrategicPlugin::PlatoonStrategicPlugin(){}
                                    

    void PlatoonStrategicPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        
        
        maneuver_srv_ = nh_->advertiseService("strategic_plan/plan_maneuvers", &PlatoonStrategicPlugin::plan_maneuver_cb, this);

        mob_op_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("mobility_operation_message", 5);
        mob_req_pub_ = nh_->advertise<cav_msgs::MobilityRequest>("mobility_request_message", 5);
                
        platoon_strategic_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatooningStrategicPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonStrategicPlugin::pose_cb, this);

        mob_req_sub_ = nh_->subscribe("incoming_mobility_request_message", 1, &PlatoonStrategicPlugin::mob_req_cb, this);
        mob_resp_sub_ = nh_->subscribe("incoming_mobility_response_message", 1, &PlatoonStrategicPlugin::mob_resp_cb, this);
        mob_op_sub_ = nh_->subscribe("incoming_mobility_operation_message", 1, &PlatoonStrategicPlugin::mob_op_cb, this);


        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            platoon_strategic_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });


    }

    void PlatoonStrategicPlugin::run_states(){
        
        switch (psm_.current_platoon_state)
        {
        case PlatoonState::STANDBY:
            break;
        case PlatoonState::LEADERWAITING:
            waitingStartTime = ros::Time::now().toSec()*1000;
            run_leader_waiting();
            break;

        case PlatoonState::LEADER:
            run_leader();
            break;
        
        case PlatoonState::CANDIDATEFOLLOWER:
            candidatestateStartTime = ros::Time::now().toSec()*1000;
            run_candidate_follower();
            break;
        
        case PlatoonState::FOLLOWER:
            run_follower();
            break;
        
        default:
            throw std::invalid_argument("Invalid State");
            break;
        }


    }

    void PlatoonStrategicPlugin::run()
    {
    	initialize();
        psm_.onMobilityRequestMessage(mobility_req_msg_);
        psm_.onMobilityOperationMessage(mobility_op_msg_);
        psm_.onMobilityResponseMessage(mobility_resp_msg_);
        run_states();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    void PlatoonStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    bool PlatoonStrategicPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp){
        
        cav_msgs::Maneuver new_maneuver = psm_.composeManeuver();
        resp.new_plan.maneuvers.push_back(new_maneuver);
        return true;
    }



    void PlatoonStrategicPlugin::run_leader_waiting(){
        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
                if(tsStart - waitingStartTime > waitingStateTimeout * 1000) {
                    //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
                    ROS_DEBUG("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
                    psm_.current_platoon_state = PlatoonState::LEADER;
                }
                // Task 2
                cav_msgs::MobilityOperation status;
                composeMobilityOperationLeaderWaiting(status);
                mob_op_pub_.publish(status);
                long tsEnd = ros::Time::now().toSec()*1000;
                int sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
    }

    void PlatoonStrategicPlugin::run_leader(){

        long tsStart = ros::Time::now().toSec()*1000;
            // Task 1
            bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval;
            if(isTimeForHeartBeat) {
                    cav_msgs::MobilityOperation infoOperation;
                    composeMobilityOperationLeader(infoOperation, "INFO");
                    mob_op_pub_.publish(infoOperation);
                    lastHeartBeatTime = ros::Time::now().toSec()*1000.0;
                    ROS_DEBUG("Published heart beat platoon INFO mobility operatrion message");
                }
            // Task 2
            if (isTimeForHeartBeat) {
                    // updateLightBar();
            }
            // Task 3
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_.current_plan.valid) {
                    bool isCurrentPlanTimeout = ((ros::Time::now().toSec()*1000 - psm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
                    if(isCurrentPlanTimeout) {
                        ROS_DEBUG("Give up current on waiting plan with planId: " , psm_.current_plan.planId);
                        psm_.current_plan.valid = false;
                    }    
                }
            }

            // Task 4
            bool hasFollower = (psm_.pm_.getTotalPlatooningSize() > 1);
            if(hasFollower) {
                cav_msgs::MobilityOperation statusOperation;
                composeMobilityOperationLeader(statusOperation, "STATUS");
                mob_op_pub_.publish(statusOperation);
                ROS_DEBUG("Published platoon STATUS operation message");
            }
            long tsEnd =  ros::Time::now().toSec()*1000;
            long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
            ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_follower(){
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request when we get close
        // 4. Publish operation status every 100 milliseconds if we still have followers
        long tsStart = ros::Time::now().toSec()*1000;
            // Job 1
            cav_msgs::MobilityOperation status;
            composeMobilityOperationFollower(status);
            mob_op_pub_.publish(status);
            // Job 2
            // Get the number of vehicles in this platoon who is in front of us
            int vehicleInFront = psm_.pm_.getNumberOfVehicleInFront();
                if(vehicleInFront == 0) {
                    noLeaderUpdatesCounter++;
                    if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
                        ROS_DEBUG("noLeaderUpdatesCounter = " , noLeaderUpdatesCounter , " and change to leader state");
                        psm_.pm_.changeFromFollowerToLeader();
                        psm_.current_platoon_state = PlatoonState::LEADER;
                        
                    }
                } else {
                    // reset counter to zero when we get updates again
                    noLeaderUpdatesCounter = 0;
                }
                long tsEnd = ros::Time::now().toSec()*1000.0;
                long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_candidate_follower(){
        long tsStart = ros::Time::now().toSec()*1000.0; 
        // Task 1
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        if(isCurrentStateTimeout) {
            ROS_DEBUG("The current candidate follower state is timeout. Change back to leader state.");
            psm_.current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2

        if(psm_.current_plan.valid) {
            {
                std::lock_guard<std::mutex> lock(plan_mutex_);
                if(psm_.current_plan.valid) {
                    bool isPlanTimeout = (tsStart - psm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
                    if(isPlanTimeout) {
                        psm_.current_plan.valid = false;
                        ROS_DEBUG("The current plan did not receive any response. Abort and change to leader state.");
                        psm_.current_platoon_state = PlatoonState::LEADER;
                    }    
                }
            }
        }

        // Task 3
                double desiredJoinGap2 = desiredJoinTimeGap;
                double maxJoinGap = std::max(desiredJoinGap, desiredJoinGap2);
                double currentGap = psm_.pm_.getDistanceToFrontVehicle();
                ROS_DEBUG("Based on desired join time gap, the desired join distance gap is " , desiredJoinGap2 , " ms");
                ROS_DEBUG("Since we have max allowed gap as " , desiredJoinGap , " m then max join gap became " , maxJoinGap , " m");
                ROS_DEBUG("The current gap from radar is " , currentGap , " m");
                if(currentGap <= maxJoinGap && psm_.current_plan.valid == false) {
                    cav_msgs::MobilityRequest request;
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toSec()*1000.0; 
                    request.header.plan_id = planId;
                    request.header.recipient_id = psm_.targetLeaderId;
                    request.header.sender_bsm_id = BSMID;
                    request.header.sender_id = MobilityId;
                    request.header.timestamp = currentTime;
                    cav_msgs::LocationECEF loc;
                    request.location = loc;
                    request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
                    std::string MOBILITY_STRATEGY;
                    request.strategy = MOBILITY_STRATEGY;
                    request.strategy_params = "";
                    request.urgency = 50;
                    mob_req_pub_.publish(request);
                    ROS_DEBUG("Published Mobility Candidate-Join request to the leader");
                    
                    PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, psm_.targetLeaderId);

                    psm_.current_plan = *new_plan;
                }
        
         //Task 4
                if(psm_.pm_.getTotalPlatooningSize() > 1) {
                    cav_msgs::MobilityOperation status;
                    composeMobilityOperationCandidateFollower(status);
                    mob_op_pub_.publish(status);
                    ROS_DEBUG("Published platoon STATUS operation message");
                }
                long tsEnd =  ros::Time::now().toSec()*1000;
                long sleepDuration = std::max(int(statusMessageInterval - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }



    void PlatoonStrategicPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        mobility_req_msg_ = msg;
    }

    void PlatoonStrategicPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {
        mobility_resp_msg_ = msg;
    }

    void PlatoonStrategicPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        mobility_op_msg_ = msg;
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeader(cav_msgs::MobilityOperation &msg, const std::string& type){
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        msg.strategy = MOBILITY_STRATEGY;

        if (type == OPERATION_INFO_TYPE){
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d

            std::string PlatoonRearBsmId = BSMID;
            int CurrentPlatoonLength = psm_.pm_.getCurrentPlatoonLength();
            double current_speed = psm_.pm_.getCurrentSpeed();
            int PlatoonSize = psm_.pm_.getTotalPlatooningSize();
            double PlatoonRearDowntrackDistance = psm_.pm_.getPlatoonRearDowntrackDistance();

            boost::format fmter(OPERATION_INFO_TYPE);
            fmter %PlatoonRearBsmId;
            fmter %CurrentPlatoonLength;
            fmter %current_speed;
            fmter %PlatoonSize;
            fmter %PlatoonRearDowntrackDistance;

            std::string infoParams = fmter.str();
            msg.strategy_params = infoParams;
        }
        else if (type == OPERATION_STATUS_TYPE){
            // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
            double cmdSpeed, current_speed, current_downtrack;
            boost::format fmter(OPERATION_STATUS_PARAMS);
            fmter %cmdSpeed;
            fmter %current_downtrack;
            fmter %current_speed;
                    
            std::string statusParams = fmter.str();
            msg.strategy_params = statusParams;
        } else {
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    
    }

    void PlatoonStrategicPlugin::composeMobilityOperationFollower(cav_msgs::MobilityOperation &msg) const{
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0;
        msg.strategy = MOBILITY_STRATEGY;
        
        double cmdSpeed, current_speed, current_downtrack;

        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


    void PlatoonStrategicPlugin::composeMobilityOperationLeaderWaiting(cav_msgs::MobilityOperation &msg) const
    {
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // This message is for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000;
        msg.strategy = MOBILITY_STRATEGY;
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"
        double cmdSpeed, current_speed, current_downtrack;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        
    }


    void PlatoonStrategicPlugin::composeMobilityOperationCandidateFollower(cav_msgs::MobilityOperation &msg)
    {
        msg.header.plan_id = psm_.pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = BSMID;
        std::string hostStaticId = HostMobilityId;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toSec()*1000.0; 
        msg.strategy = MOBILITY_STRATEGY;
        
        // // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        
        double cmdSpeed, current_speed, current_downtrack;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack;
        fmter %current_speed;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG("Composed a mobility operation message with params " , msg.strategy_params);
    }


}
