
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
#include <ros/ros.h>
#include <string>
#include "platoon_strategic.hpp"


namespace platoon_strategic
{
    PlatoonStrategicPlugin::PlatoonStrategicPlugin()
    {
    }

    PlatoonStrategicPlugin::PlatoonStrategicPlugin(carma_wm::WorldModelConstPtr wm, PlatoonPluginConfig config,
                                PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher,
                                MobilityRequestCB mobility_request_publisher, MobilityOperationCB mobility_operation_publisher,
                                PlatooningInfoCB platooning_info_publisher)
    : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher), 
      mobility_response_publisher_(mobility_response_publisher), mobility_request_publisher_(mobility_request_publisher),
      mobility_operation_publisher_(mobility_operation_publisher), platooning_info_publisher_(platooning_info_publisher)
    {
        pm_ = PlatoonManager();

        plugin_discovery_msg_.name = "PlatooningStrategicPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";  
    }
                                    


    bool PlatoonStrategicPlugin::onSpin() 
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        
        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            run_leader();
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            run_follower();
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            run_candidate_follower();
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            run_leader_waiting();
        }
        // else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        // {
        //     run_standby();
        // }

        cav_msgs::PlatooningInfo platoon_status = compose_platoon_info_msg();
        platooning_info_publisher_(platoon_status);

        return true;
    }

    



    int PlatoonStrategicPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    void PlatoonStrategicPlugin::bsm_cb(const cav_msgs::BSMConstPtr& msg)
    {
        cav_msgs::BSMCoreData bsm_core_ = msg->core_data;
        host_bsm_id_ = bsmIDtoString(bsm_core_);
    }

    void PlatoonStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
            carma_wm::TrackPos tc = wm_->routeTrackPos(current_loc);
            current_downtrack_ = tc.downtrack;
            ROS_DEBUG_STREAM("current_downtrack_ = " << current_downtrack_);
            current_crosstrack_ = tc.crosstrack;
            ROS_DEBUG_STREAM("current_crosstrack_ = " << current_crosstrack_);

            pose_ecef_point_ = pose_to_ecef(pose_msg_);
        }
        
        

    }

    void PlatoonStrategicPlugin::cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        cmd_speed_ = msg->twist.linear.x;
    }

    void PlatoonStrategicPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        if (current_speed_ < 0.01)
        {
            current_speed_ = 0.0;
        }
    }

    void PlatoonStrategicPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, double& speed, double& current_progress, int& lane_id){
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            lane_id =  stoi(maneuver.lane_following_maneuver.lane_id);
        }
    }

    

    bool PlatoonStrategicPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp)
    {

        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10);       
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }
        
        auto shortest_path = wm_->getRoute()->shortestPath();

        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d()))
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path);
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    current_lanelet = shortest_path[last_lanelet_index];
                    break;
                }
            }
        }
        if(last_lanelet_index == -1)
        {
            ROS_ERROR_STREAM("Current position is not on the shortest path! Returning an empty maneuver");
            return true;
        }
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        double target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit TOTO update

        double total_maneuver_length = current_progress + config_.mvr_duration * target_speed;
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);

        //Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!=0){
            time_progress = req.prior_plan.planning_completion_time;
            int end_lanelet =0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(),speed_progress,current_progress,end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet,shortest_path);
        }
        bool approaching_route_end = false;
        double time_req_to_stop,stopping_dist;

        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        while(current_progress < total_maneuver_length)
        {
            ROS_DEBUG_STREAM("Lanlet: " << shortest_path[last_lanelet_index].id());
            ROS_DEBUG_STREAM("current_progress: "<< current_progress);
            ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
            ROS_DEBUG_STREAM("target_speed: " << target_speed);
            ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
            auto p = shortest_path[last_lanelet_index].centerline2d().back();
            double end_dist = wm_->routeTrackPos(shortest_path[last_lanelet_index].centerline2d().back()).downtrack;
            end_dist = std::min(end_dist, total_maneuver_length);
            ROS_DEBUG_STREAM("end_dist: " << end_dist);
            double dist_diff = end_dist - current_progress;
            ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
            if(end_dist < current_progress){
                break;
            }

            resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                    speed_progress, target_speed,shortest_path[last_lanelet_index].id(), time_progress));

            
            current_progress += dist_diff;
            time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
            speed_progress = target_speed;
            if(current_progress >= total_maneuver_length || last_lanelet_index == shortest_path.size() - 1)
            {
                break;
            }
            ++last_lanelet_index;
        }

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }  

        if (pm_.getTotalPlatooningSize() < 2)
        {
            resp.new_plan.maneuvers = {};
            ROS_WARN_STREAM("Platoon size 1 so Empty maneuver sent");
        }

        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            pm_.current_platoon_state = PlatoonState::LEADER;
            pm_.currentPlatoonID = boost::uuids::to_string(boost::uuids::random_generator()());
            ROS_DEBUG_STREAM("change the state from standby to leader at start-up");
        }

        
        pm_.current_downtrack_didtance_ = current_downtrack_;
        pm_.HostMobilityId = config_.vehicle_id;
        ROS_DEBUG_STREAM("current_downtrack: " << current_downtrack_);
        
        return true;
    }

    double PlatoonStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        double target_speed = 0.0, traffic_speed =0.0, param_speed =0.0;
        double hardcoded_max=lanelet::Velocity(hardcoded_params::control_limits::MAX_LONGITUDINAL_VELOCITY_MPS * lanelet::units::MPS()).value();

        if (traffic_rules)
        {
            traffic_speed=(*traffic_rules)->speedLimit(llt).speedLimit.value();
            
        }
        else{
            ROS_WARN(" Valid traffic rules object could not be built.");
        }

        if(config_.config_limit > 0.0 && config_.config_limit < hardcoded_max)
        {
            param_speed = config_.config_limit;
            ROS_DEBUG_STREAM("Using Configurable value");
        }
        else 
        {
            param_speed = hardcoded_max;
            ROS_DEBUG_STREAM(" Using Hardcoded maximum");
        }
        //If either value is 0, use the other valid limit
        if(traffic_speed <= config_.epislon || param_speed <= config_.epislon){
            target_speed = std::max(traffic_speed, param_speed);
        }
        else{
            target_speed = std::min(traffic_speed,param_speed);
        }
        
        return target_speed;
    }

    cav_msgs::Maneuver PlatoonStrategicPlugin::composeManeuverMessage(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::PLATOONING;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "PlatooningTacticalPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "PlatooningStrategicPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.mvr_duration);
        } else {
            maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
        }
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_DEBUG_STREAM("Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM("Duration:"<< maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }



    void PlatoonStrategicPlugin::run_leader_waiting(){
        ROS_DEBUG_STREAM("Run LeaderWaiting State ");
        long tsStart = ros::Time::now().toNSec()/1000000;
            // Task 1
                if(tsStart - waitingStartTime > waitingStateTimeout * 1000) 
                {
                    //TODO if the current state timeouts, we need to have a kind of ABORT message to inform the applicant
                    ROS_DEBUG_STREAM("LeaderWaitingState is timeout, changing back to PlatoonLeaderState.");
                    pm_.current_platoon_state = PlatoonState::LEADER;
                }
                // Task 2
                cav_msgs::MobilityOperation status;
                status = composeMobilityOperationLeaderWaiting();
                std::cerr << status << std::endl;
                mobility_operation_publisher_(status);
                ROS_DEBUG_STREAM("publish status message");
                long tsEnd = ros::Time::now().toNSec()/1000000; 
                long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
    }

    void PlatoonStrategicPlugin::run_leader(){

        long tsStart = ros::Time::now().toNSec()/1000000; 
            // Task 1
            bool isTimeForHeartBeat = tsStart - lastHeartBeatTime >= infoMessageInterval_;
            ROS_DEBUG_STREAM("time since last heart beat: " << tsStart - lastHeartBeatTime);
            if(isTimeForHeartBeat) {
                    cav_msgs::MobilityOperation infoOperation;
                    infoOperation = composeMobilityOperationLeader(OPERATION_INFO_TYPE);
                    mobility_operation_publisher_(infoOperation);
                    lastHeartBeatTime = ros::Time::now().toNSec()/1000000; 
                    ROS_DEBUG_STREAM("Published heart beat platoon INFO mobility operatrion message");
                }
            // Task 2
            // if (isTimeForHeartBeat) {
            //     updateLightBar();
            // }
            // Task 3
                if(pm_.current_plan.valid)
                {
                    bool isCurrentPlanTimeout = ((ros::Time::now().toNSec()/1000000  - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT);
                    if(isCurrentPlanTimeout) 
                    {
                        ROS_DEBUG_STREAM("Give up current on waiting plan with planId: " << pm_.current_plan.planId);
                        pm_.current_plan.valid = false;
                    }    
                }

            // Task 4
            bool hasFollower = (pm_.getTotalPlatooningSize() > 1);
            if(hasFollower) {
                cav_msgs::MobilityOperation statusOperation;
                statusOperation = composeMobilityOperationLeader(OPERATION_STATUS_TYPE);
                // mob_op_pub_.publish(statusOperation);
                mobility_operation_publisher_(statusOperation);
                ROS_DEBUG_STREAM("Published platoon STATUS operation message");
            }
            long tsEnd =  ros::Time::now().toNSec()/1000000; 
            long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
            ros::Duration(sleepDuration/1000).sleep();
        
    }

    void PlatoonStrategicPlugin::run_follower(){
        // This is a interrupted-safe loop.
        // This loop has four tasks:
        // 1. Check the state start time, if it exceeds a limit it will give up current plan and change back to leader state
        // 2. Abort current request if we wait for long enough time for response from leader and change back to leader state
        // 3. Check the current distance with the target platoon rear and send out CANDIDATE-JOIN request when we get close
        // 4. Publish operation status every 100 milliseconds if we still have followers
        long tsStart = ros::Time::now().toNSec()/1000000; 
            // Job 1
            cav_msgs::MobilityOperation status;
            status = composeMobilityOperationFollower();
            mobility_operation_publisher_(status);
            // Job 2
            // Get the number of vehicles in this platoon who is in front of us
            int vehicleInFront = pm_.getNumberOfVehicleInFront();
                if(vehicleInFront == 0) {
                    noLeaderUpdatesCounter++;
                    if(noLeaderUpdatesCounter >= LEADER_TIMEOUT_COUNTER_LIMIT) {
                        ROS_DEBUG_STREAM("noLeaderUpdatesCounter = " << noLeaderUpdatesCounter << " and change to leader state");
                        pm_.changeFromFollowerToLeader();
                        pm_.current_platoon_state = PlatoonState::LEADER;
                    }
                } else {
                    // reset counter to zero when we get updates again
                    noLeaderUpdatesCounter = 0;
                }
                long tsEnd = ros::Time::now().toNSec()/1000000;
                long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();        
    }

    void PlatoonStrategicPlugin::run_candidate_follower(){
        long tsStart = ros::Time::now().toNSec()/1000000;
        // Task 1
        bool isCurrentStateTimeout = (tsStart - candidatestateStartTime) > waitingStateTimeout * 1000;
        ROS_DEBUG_STREAM("timeout1: " << tsStart - candidatestateStartTime);
        ROS_DEBUG_STREAM("waitingStateTimeout: " << waitingStateTimeout*1000);
        if(isCurrentStateTimeout) {
            ROS_DEBUG_STREAM("The current candidate follower state is timeout. Change back to leader state.");
            pm_.current_platoon_state = PlatoonState::LEADER;
        }
        // Task 2

        if(pm_.current_plan.valid) {
            {
                if(pm_.current_plan.valid) {
                    ROS_DEBUG_STREAM("pm_.current_plan.planStartTime: " << pm_.current_plan.planStartTime);
                    ROS_DEBUG_STREAM("timeout2: " << tsStart - pm_.current_plan.planStartTime);
                    ROS_DEBUG_STREAM("NEGOTIATION_TIMEOUT: " << NEGOTIATION_TIMEOUT);
                    bool isPlanTimeout = (tsStart - pm_.current_plan.planStartTime) > NEGOTIATION_TIMEOUT;
                    if(isPlanTimeout) {
                        pm_.current_plan.valid = false;
                        ROS_DEBUG_STREAM("The current plan did not receive any response. Abort and change to leader state.");
                        pm_.current_platoon_state = PlatoonState::LEADER;
                        ROS_DEBUG_STREAM("Changed the state back to Leader");
                    }    
                }
            }
        }

        // Task 3
                double desiredJoinGap2 = config_.desiredJoinTimeGap * current_speed_;
                double maxJoinGap = std::max(config_.desiredJoinGap, desiredJoinGap2);
                double currentGap = pm_.getDistanceToFrontVehicle();
                ROS_DEBUG_STREAM("Based on desired join time gap, the desired join distance gap is " << desiredJoinGap2 << " ms");
                ROS_DEBUG_STREAM("Since we have max allowed gap as " << desiredJoinGap << " m then max join gap became " << maxJoinGap << " m");
                ROS_DEBUG_STREAM("The current gap from radar is " << currentGap << " m");
                // TODO: temporary
                if(true)//(currentGap <= maxJoinGap && pm_.current_plan.valid == false) {
                {
                    cav_msgs::MobilityRequest request;
                    std::string planId = boost::uuids::to_string(boost::uuids::random_generator()());
                    long currentTime = ros::Time::now().toNSec()/1000000; 
                    request.header.plan_id = planId;
                    request.header.recipient_id = pm_.targetLeaderId;
                    request.header.sender_bsm_id = host_bsm_id_;
                    request.header.sender_id = config_.vehicle_id;
                    request.header.timestamp = currentTime;
                    request.plan_type.type = cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;
                    request.strategy = MOBILITY_STRATEGY;
                    request.strategy_params = "";
                    request.urgency = 50;
                    request.location = pose_to_ecef(pose_msg_);
                    mobility_request_publisher_(request);
                    ROS_DEBUG_STREAM("Published Mobility Candidate-Join request to the leader");
                    ROS_WARN("Published Mobility Candidate-Join request to the leader");
                    PlatoonPlan* new_plan = new PlatoonPlan(true, currentTime, planId, pm_.targetLeaderId);

                    pm_.current_plan = *new_plan;
                }
        
         //Task 4
                if(pm_.getTotalPlatooningSize() > 1) {
                    cav_msgs::MobilityOperation status;
                    status = composeMobilityOperationCandidateFollower();
                    mobility_operation_publisher_(status);
                    ROS_DEBUG_STREAM("Published platoon STATUS operation message");
                }
                long tsEnd =  ros::Time::now().toNSec()/1000000; 
                long sleepDuration = std::max((int32_t)(statusMessageInterval_ - (tsEnd - tsStart)), 0);
                ros::Duration(sleepDuration/1000).sleep();
        
    }

    MobilityRequestResponse PlatoonStrategicPlugin::handle_mob_req(const cav_msgs::MobilityRequest& msg)
    {
        MobilityRequestResponse mobility_response;
        

        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mobility_response = mob_req_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mobility_response = mob_req_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mobility_response = mob_req_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mobility_response = mob_req_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mobility_response = mob_req_cb_standby(msg);
        }

        return mobility_response;

    }

    void PlatoonStrategicPlugin::mob_req_cb(const cav_msgs::MobilityRequest& msg)
    {
        cav_msgs::MobilityResponse response;
        response.header.sender_id = config_.vehicle_id;
        response.header.recipient_id = msg.header.sender_id;
        response.header.plan_id = pm_.currentPlatoonID;
        response.header.sender_bsm_id = host_bsm_id_;
        response.header.timestamp = ros::Time::now().toNSec()/1000000;
        MobilityRequestResponse req_response = handle_mob_req(msg);
        if (req_response == MobilityRequestResponse::ACK)
        {
            response.is_accepted = true;
            mobility_response_publisher_(response);
        }
        else if (req_response == MobilityRequestResponse::NACK)
        {
            response.is_accepted = false;
            mobility_response_publisher_(response);
        }
        else
        {
            ROS_DEBUG_STREAM(" NO response to mobility request. ");
        }
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_standby(const cav_msgs::MobilityRequest& msg)
    {
        // In standby state, the plugin is not responsible for replying to any request messages
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_candidatefollower(const cav_msgs::MobilityRequest& msg)
    {
        // This state does not handle any mobility request for now
        // TODO Maybe it should handle some ABORT request from a waiting leader
        ROS_DEBUG_STREAM("Recived mobility request with type " << msg.plan_type.type << " but ignored.");
        return MobilityRequestResponse::NO_RESPONSE;

    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_follower(const cav_msgs::MobilityRequest& msg)
    {
        return MobilityRequestResponse::NO_RESPONSE;
    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_leaderwaiting(const cav_msgs::MobilityRequest& msg)
    {

        bool isTargetVehicle = (msg.header.sender_id == lw_applicantId_);
        bool isCandidateJoin = msg.plan_type.type == cav_msgs::PlanType::PLATOON_FOLLOWER_JOIN;

        lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
        double obj_cross_track = wm_->routeTrackPos(incoming_pose).crosstrack;
        bool inTheSameLane = (abs(obj_cross_track - current_crosstrack_) < config_.maxCrosstrackError);
        ROS_DEBUG_STREAM("current_cross_track error = " << abs(obj_cross_track - current_crosstrack_));
        ROS_DEBUG_STREAM("inTheSameLane = " << inTheSameLane);
        if(isTargetVehicle && isCandidateJoin && inTheSameLane)
        {
            ROS_DEBUG_STREAM("Target vehicle " << lw_applicantId_ << " is actually joining.");
            ROS_DEBUG_STREAM("Changing to PlatoonLeaderState and send ACK to target vehicle");
            pm_.current_platoon_state = PlatoonState::LEADER;
            return MobilityRequestResponse::ACK;
        }
        else 
        {
            ROS_DEBUG_STREAM("Received platoon request with vehicle id = " << msg.header.sender_id);
            ROS_DEBUG_STREAM("The request type is " << msg.plan_type.type << " and we choose to ignore");
            return MobilityRequestResponse::NO_RESPONSE;
        }

    }

    MobilityRequestResponse PlatoonStrategicPlugin::mob_req_cb_leader(const cav_msgs::MobilityRequest& msg)
    {
        mobility_req_msg_ = msg;
        cav_msgs::PlanType plan_type= msg.plan_type;
        if (plan_type.type == cav_msgs::PlanType::JOIN_PLATOON_AT_REAR)
        {
            // We are currently checking two basic JOIN conditions:
            //     1. The size limitation on current platoon based on the plugin's parameters.
            //     2. Calculate how long that vehicle can be in a reasonable distance to actually join us.
            // TODO We ignore the lane information for now and assume the applicant is in the same lane with us.
            cav_msgs::MobilityHeader msgHeader = msg.header;
            std::string params = msg.strategy_params;
            std::string applicantId = msgHeader.sender_id;
            ROS_DEBUG_STREAM("Receive mobility JOIN request from " << applicantId << " and PlanId = " << msgHeader.plan_id);
            ROS_DEBUG_STREAM("The strategy parameters are " << params);
            if (params == "")
            {
                ROS_DEBUG_STREAM("The strategy parameters are empty, return no response");
                return MobilityRequestResponse::NO_RESPONSE;
            }
            // For JOIN_PLATOON_AT_REAR message, the strategy params is defined as "SIZE:xx,SPEED:xx,DTD:xx"
            // TODO In future, we should remove down track distance from this string and use location field in request message
            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, params, boost::is_any_of(","));
            
            std::vector<std::string> applicantSize_parsed;
            boost::algorithm::split(applicantSize_parsed, inputsParams[0], boost::is_any_of(":"));
            int applicantSize = std::stoi(applicantSize_parsed[1]);
            ROS_DEBUG_STREAM("applicantSize: " << applicantSize);
            
            std::vector<std::string> applicantCurrentSpeed_parsed;
            boost::algorithm::split(applicantCurrentSpeed_parsed, inputsParams[1], boost::is_any_of(":"));
            double applicantCurrentSpeed = std::stod(applicantCurrentSpeed_parsed[1]);
            ROS_DEBUG_STREAM("applicantCurrentSpeed: " << applicantCurrentSpeed);
            
            std::vector<std::string> applicantCurrentDtd_parsed;
            boost::algorithm::split(applicantCurrentDtd_parsed, inputsParams[2], boost::is_any_of(":"));
            double applicantCurrentDtd = std::stod(applicantCurrentDtd_parsed[1]);
            ROS_DEBUG_STREAM("applicantCurrentDtd from message: " << applicantCurrentDtd);

            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(msg.location);
            applicantCurrentDtd = wm_->routeTrackPos(incoming_pose).downtrack;
            ROS_DEBUG_STREAM("applicantCurrentmemberUpdates from ecef pose: " << applicantCurrentDtd);

            double applicant_crosstrack = wm_->routeTrackPos(incoming_pose).crosstrack;
            ROS_DEBUG_STREAM("applicant_crosstrack from ecef pose: " << applicant_crosstrack);
            bool isInLane = (abs(applicant_crosstrack - current_crosstrack_) < config_.maxCrosstrackError);
            ROS_DEBUG_STREAM("isInLane = " << isInLane);
            // Check if we have enough room for that applicant
            int currentPlatoonSize = pm_.getTotalPlatooningSize();
            bool hasEnoughRoomInPlatoon = applicantSize + currentPlatoonSize <= maxPlatoonSize_;
            if(hasEnoughRoomInPlatoon && isInLane) {
                ROS_DEBUG_STREAM("The current platoon has enough room for the applicant with size " << applicantSize);
                double currentRearDtd = pm_.getPlatoonRearDowntrackDistance();
                ROS_DEBUG_STREAM("The current platoon rear dtd is " << currentRearDtd);
                double currentGap = currentRearDtd - applicantCurrentDtd - config_.vehicleLength;
                double currentTimeGap = currentGap / applicantCurrentSpeed;
                ROS_DEBUG_STREAM("The gap between current platoon rear and applicant is " << currentGap << "m or " << currentTimeGap << "s");
                if(currentGap < 0) {
                    ROS_WARN("We should not receive any request from the vehicle in front of us. NACK it.");
                    return MobilityRequestResponse::NACK;
                }
                // Check if the applicant can join based on max timeGap/gap
                bool isDistanceCloseEnough = (currentGap <= maxAllowedJoinGap_) || (currentTimeGap <= maxAllowedJoinTimeGap_);
                if(isDistanceCloseEnough) {
                    ROS_DEBUG_STREAM("The applicant is close enough and we will allow it to try to join");
                    ROS_DEBUG_STREAM("Change to LeaderWaitingState and waiting for " << msg.header.sender_id << " to join");
                    pm_.current_platoon_state = PlatoonState::LEADERWAITING;
                    waitingStartTime = ros::Time::now().toNSec()/1000000;
                    lw_applicantId_ = msg.header.sender_id;
                    // plugin.setState(new LeaderWaitingState(plugin, log, pluginServiceLocator, applicantId));
                    return MobilityRequestResponse::ACK;
                } else {
                    ROS_DEBUG_STREAM("The applicant is too far away from us. NACK.");
                    return MobilityRequestResponse::NACK;
                }
            }
            else 
            {
                ROS_DEBUG_STREAM("The current platoon does not have enough room for applicant of size " << applicantSize << ". NACK");
                return MobilityRequestResponse::NACK;
            } 
        } 
        else {
            ROS_DEBUG_STREAM("Received mobility request with type " << msg.plan_type.type << " and ignored.");
            return MobilityRequestResponse::NO_RESPONSE;
        }
    }

    void PlatoonStrategicPlugin::mob_resp_cb(const cav_msgs::MobilityResponse& msg)
    {
        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mob_resp_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mob_resp_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mob_resp_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mob_resp_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mob_resp_cb_standby(msg);
        }
    }

    void PlatoonStrategicPlugin::mob_resp_cb_standby(const cav_msgs::MobilityResponse& msg)
    {
        // In standby state, it will not send out any requests so it will also ignore all responses
    }


    void PlatoonStrategicPlugin::mob_resp_cb_candidatefollower(const cav_msgs::MobilityResponse& msg)
    {
        ROS_DEBUG_STREAM("Callback for candidate follower ");
        if (pm_.current_plan.valid)
        {
            bool isForCurrentPlan = msg.header.plan_id == pm_.current_plan.planId;
            bool isFromTargetVehicle = msg.header.sender_id == pm_.targetLeaderId;
            ROS_DEBUG_STREAM("isForCurrentPlan " << isForCurrentPlan);

            ROS_DEBUG_STREAM("isFromTargetVehicle " << isFromTargetVehicle);

            //if (isForCurrentPlan && isFromTargetVehicle)  //TODO: This check not needed for now
            if (true)
            {
                if(msg.is_accepted) 
                {
                    // We change to follower state and start to actually follow that leader
                    // The platoon manager also need to change the platoon Id to the one that the target leader is using 
                    ROS_DEBUG_STREAM("The leader " << msg.header.sender_id << " agreed on our join. Change to follower state.");
                    pm_.current_platoon_state = PlatoonState::FOLLOWER;
                    targetPlatoonId = msg.header.plan_id;
                    pm_.changeFromLeaderToFollower(targetPlatoonId);
                    ROS_WARN("changed to follower");
                    
                } 
                else
                {
                    // We change back to normal leader state and try to join other platoons
                    ROS_DEBUG_STREAM("The leader " << msg.header.sender_id << " does not agree on our join. Change back to leader state.");
                    pm_.current_platoon_state = PlatoonState::LEADER;
                }
            }
            else 
            {
                ROS_DEBUG_STREAM("Ignore received response message because it is not for the current plan.");
            }  
        }
        else 
        {
            ROS_DEBUG_STREAM("Ignore received response message because we are not in any negotiation process.");
        }
    }

    void PlatoonStrategicPlugin::mob_resp_cb_leaderwaiting(const cav_msgs::MobilityResponse& msg)
    {
    }
    
    void PlatoonStrategicPlugin::mob_resp_cb_follower(const cav_msgs::MobilityResponse& msg)
    {
    }

    void PlatoonStrategicPlugin::mob_resp_cb_leader(const cav_msgs::MobilityResponse& msg)
    {
        if (pm_.current_plan.valid)
        {
            // if (pm_.current_plan.planId == msg.header.plan_id && pm_.current_plan.peerId == msg.header.sender_id) //TODO this check not needed here, 
            if (true)
            {
                if (msg.is_accepted)
                {
                    ROS_DEBUG_STREAM("Received positive response for plan id = " << pm_.current_plan.planId);
                    ROS_DEBUG_STREAM("Change to CandidateFollower state and notify trajectory failure in order to replan");
                        // Change to candidate follower state and request a new plan to catch up with the front platoon
                        pm_.current_platoon_state = PlatoonState::CANDIDATEFOLLOWER;
                        candidatestateStartTime = ros::Time::now().toNSec()/1000000;
                        targetPlatoonId = potentialNewPlatoonId;
                        ROS_DEBUG_STREAM("targetPlatoonId = " << targetPlatoonId);
                        pm_.targetLeaderId = pm_.current_plan.peerId;
                        ROS_DEBUG_STREAM("pm_.targetLeaderId = " << pm_.targetLeaderId );
                }
                else 
                {
                    ROS_DEBUG_STREAM("Received negative response for plan id = " << pm_.current_plan.planId);
                    // Forget about the previous plan totally
                    pm_.current_plan.valid = false;
                }
            }
            else
            {
                ROS_DEBUG_STREAM("Ignore the response message because planID match: " << (pm_.current_plan.planId == msg.header.plan_id));
                ROS_DEBUG_STREAM("My plan id = " << pm_.current_plan.planId << " and response plan Id = " << msg.header.plan_id);
                ROS_DEBUG_STREAM("And peer id match " << (pm_.current_plan.peerId == msg.header.sender_id));
                ROS_DEBUG_STREAM("Expected peer id = " << pm_.current_plan.peerId << " and response sender Id = " << msg.header.sender_id);
            }
        }
    }

    cav_msgs::PlatooningInfo PlatoonStrategicPlugin::compose_platoon_info_msg()
    {
        cav_msgs::PlatooningInfo status_msg;

        if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            status_msg.state = cav_msgs::PlatooningInfo::DISABLED;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            status_msg.state = pm_.getTotalPlatooningSize() == 1 ? cav_msgs::PlatooningInfo::SEARCHING : cav_msgs::PlatooningInfo::LEADING;
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_FOLLOWER;
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            status_msg.state = cav_msgs::PlatooningInfo::CONNECTING_TO_NEW_LEADER;
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            status_msg.state = cav_msgs::PlatooningInfo::FOLLOWING;
        }

        if (pm_.current_platoon_state != PlatoonState::STANDBY)
        {
            status_msg.platoon_id = pm_.currentPlatoonID;
            status_msg.size = std::max(1, pm_.getTotalPlatooningSize());
            status_msg.size_limit = config_.maxPlatoonSize;

            
            
            if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
            {  
                ROS_DEBUG_STREAM("isFollower: " << pm_.isFollower);
                ROS_DEBUG_STREAM("pm platoonsize: " << pm_.platoon.size());

                pm_.isFollower = true;
                
                PlatoonMember platoon_leader = pm_.getLeader();
                ROS_DEBUG_STREAM("platoon_leader " << platoon_leader.staticId);
                status_msg.leader_id = platoon_leader.staticId;
                status_msg.leader_downtrack_distance = platoon_leader.vehiclePosition;
                ROS_DEBUG_STREAM("platoon_leader position: " << platoon_leader.vehiclePosition);
                status_msg.leader_cmd_speed = platoon_leader.commandSpeed;
                status_msg.host_platoon_position = pm_.getNumberOfVehicleInFront();

                status_msg.desired_gap = std::max(config_.standStillHeadway, config_.timeHeadway *  current_speed_);
                status_msg.actual_gap = platoon_leader.vehiclePosition - current_downtrack_;

            }
            else
            {
                status_msg.leader_id = config_.vehicle_id;
                status_msg.leader_downtrack_distance = current_downtrack_;
                status_msg.leader_cmd_speed = cmd_speed_;
                status_msg.host_platoon_position = 0;
   
            }
            
            // This info is updated at platoon control plugin
            status_msg.host_cmd_speed = cmd_speed_;
            

        }
        return status_msg;
    }

    void PlatoonStrategicPlugin::mob_op_cb(const cav_msgs::MobilityOperation& msg)
    {
        if (pm_.current_platoon_state == PlatoonState::LEADER)
        {
            mob_op_cb_leader(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::FOLLOWER)
        {
            mob_op_cb_follower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::CANDIDATEFOLLOWER)
        {
            mob_op_cb_candidatefollower(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::LEADERWAITING)
        {
            mob_op_cb_leaderwaiting(msg);
        }
        else if (pm_.current_platoon_state == PlatoonState::STANDBY)
        {
            mob_op_cb_standby(msg);
        }
    

        // TODO: If needed (with large size platoons), add a queue for status messages
        // INFO messages always processed, STATUS messages if saved in que
        
    }

    void PlatoonStrategicPlugin::mob_op_cb_standby(const cav_msgs::MobilityOperation& msg)
    {
        // In standby state, it will ignore operation message since it is not actively operating
    }

    void PlatoonStrategicPlugin::mob_op_cb_candidatefollower(const cav_msgs::MobilityOperation& msg)
    {
        // We still need to handle STATUS operAtion message from our platoon
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);

            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> ecef_x_parsed;
            boost::algorithm::split(ecef_x_parsed, inputsParams[3], boost::is_any_of(":"));
            double ecef_x = std::stod(ecef_x_parsed[1]);
            ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);

            std::vector<std::string> ecef_y_parsed;
            boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
            double ecef_y = std::stod(ecef_y_parsed[1]);
            ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);

            std::vector<std::string> ecef_z_parsed;
            boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
            double ecef_z = std::stod(ecef_z_parsed[1]);
            ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
            
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc.ecef_x = ecef_x;
            ecef_loc.ecef_y = ecef_y;
            ecef_loc.ecef_z = ecef_z;
            
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double dtd = wm_->routeTrackPos(incoming_pose).downtrack;

            pm_.memberUpdates(vehicleID, platoonId, msg.header.sender_bsm_id, statusParams, dtd);
            ROS_DEBUG_STREAM("Received platoon status message from " << msg.header.sender_id);
        }
        else {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_follower(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        // In the current state, we care about the STATUS message
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);
        // If it is platoon status message, the params string is in format:
        // STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonID = msg.header.plan_id;
            std::string senderBSM = msg.header.sender_bsm_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);
            ROS_DEBUG_STREAM("Receive operation message from vehicle: " << vehicleID);

            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> ecef_x_parsed;
            boost::algorithm::split(ecef_x_parsed, inputsParams[3], boost::is_any_of(":"));
            double ecef_x = std::stod(ecef_x_parsed[1]);
            ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);

            std::vector<std::string> ecef_y_parsed;
            boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
            double ecef_y = std::stod(ecef_y_parsed[1]);
            ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);

            std::vector<std::string> ecef_z_parsed;
            boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
            double ecef_z = std::stod(ecef_z_parsed[1]);
            ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
            
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc.ecef_x = ecef_x;
            ecef_loc.ecef_y = ecef_y;
            ecef_loc.ecef_z = ecef_z;
            
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double dtd = wm_->routeTrackPos(incoming_pose).downtrack;
            ROS_DEBUG_STREAM("DTD calculated in mob_op_cb_follower: " << dtd);

            pm_.memberUpdates(vehicleID, platoonID, senderBSM, statusParams, dtd);
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_leaderwaiting(const cav_msgs::MobilityOperation& msg)
    {
        std::string strategyParams = msg.strategy_params;
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        if(isPlatoonStatusMsg) {
            std::string vehicleID = msg.header.sender_id;
            std::string platoonId = msg.header.plan_id;
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.size() + 1);


            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> ecef_x_parsed;
            boost::algorithm::split(ecef_x_parsed, inputsParams[3], boost::is_any_of(":"));
            double ecef_x = std::stod(ecef_x_parsed[1]);
            ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);

            std::vector<std::string> ecef_y_parsed;
            boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
            double ecef_y = std::stod(ecef_y_parsed[1]);
            ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);

            std::vector<std::string> ecef_z_parsed;
            boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
            double ecef_z = std::stod(ecef_z_parsed[1]);
            ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
            
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc.ecef_x = ecef_x;
            ecef_loc.ecef_y = ecef_y;
            ecef_loc.ecef_z = ecef_z;

            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            double dtd = wm_->routeTrackPos(incoming_pose).downtrack;

            pm_.memberUpdates(vehicleID, platoonId, msg.header.sender_bsm_id, statusParams, dtd);
            ROS_DEBUG_STREAM("Received platoon status message from " << msg.header.sender_id);
            ROS_DEBUG_STREAM("member updated");
        } else {
            ROS_DEBUG_STREAM("Received a mobility operation message with params " << msg.strategy_params << " but ignored.");
        }
    }

    void PlatoonStrategicPlugin::mob_op_cb_leader(const cav_msgs::MobilityOperation& msg)
    {   
        std::string strategyParams = msg.strategy_params;
        std::string senderId = msg.header.sender_id;
        std::string platoonId = msg.header.plan_id;
        // In the current state, we care about the INFO heart-beat operation message if we are not currently in
        // a negotiation, and also we need to care about operation from members in our current platoon

        bool isPlatoonInfoMsg = (strategyParams.rfind(OPERATION_INFO_TYPE, 0) == 0);
        bool isPlatoonStatusMsg = (strategyParams.rfind(OPERATION_STATUS_TYPE, 0) == 0);
        bool isNotInNegotiation = (pm_.current_plan.valid == false);
        if(isPlatoonInfoMsg && isNotInNegotiation)
        {
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f

            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> rearVehicleBsmId_parsed;
            boost::algorithm::split(rearVehicleBsmId_parsed, inputsParams[0], boost::is_any_of(":"));
            std::string rearVehicleBsmId = rearVehicleBsmId_parsed[1];
            ROS_DEBUG_STREAM("rearVehicleBsmId: " << rearVehicleBsmId);

            std::vector<std::string> rearVehicleDtd_parsed;
            boost::algorithm::split(rearVehicleDtd_parsed, inputsParams[4], boost::is_any_of(":"));
            double rearVehicleDtd = std::stod(rearVehicleDtd_parsed[1]);

            
            ROS_DEBUG_STREAM("rearVehicleDtd from message: " << rearVehicleDtd);

            std::vector<std::string> ecef_x_parsed;
            boost::algorithm::split(ecef_x_parsed, inputsParams[5], boost::is_any_of(":"));
            double ecef_x = std::stod(ecef_x_parsed[1]);
            ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);
            std::cerr << "ecef_x_parsed: " << ecef_x << std::endl;

            std::vector<std::string> ecef_y_parsed;
            boost::algorithm::split(ecef_y_parsed, inputsParams[6], boost::is_any_of(":"));
            double ecef_y = std::stod(ecef_y_parsed[1]);
            ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);
            std::cerr << "ecef_y_parsed: " << ecef_y << std::endl;

            std::vector<std::string> ecef_z_parsed;
            boost::algorithm::split(ecef_z_parsed, inputsParams[7], boost::is_any_of(":"));
            double ecef_z = std::stod(ecef_z_parsed[1]);
            ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
            std::cerr << "ecef_z_parsed: " << ecef_z << std::endl;
            
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc.ecef_x = ecef_x;
            ecef_loc.ecef_y = ecef_y;
            ecef_loc.ecef_z = ecef_z;

            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            rearVehicleDtd = wm_->routeTrackPos(incoming_pose).downtrack;

            ROS_DEBUG_STREAM("rearVehicleDtd from ecef: " << rearVehicleDtd);
            
            // We are trying to validate is the platoon rear is right in front of the host vehicle
            if(isVehicleRightInFront(rearVehicleBsmId, rearVehicleDtd))
            {
                ROS_DEBUG_STREAM("Found a platoon with id = " << platoonId << " in front of us.");
                cav_msgs::MobilityRequest request;
                request.header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
                request.header.recipient_id = senderId;
                request.header.sender_bsm_id = host_bsm_id_;
                request.header.sender_id = config_.vehicle_id;
                request.header.timestamp = ros::Time::now().toNSec()/1000000;
                request.location = pose_to_ecef(pose_msg_);
                request.plan_type.type = cav_msgs::PlanType::JOIN_PLATOON_AT_REAR;
                request.strategy = MOBILITY_STRATEGY;

                int platoon_size = pm_.getTotalPlatooningSize();
                

                boost::format fmter(JOIN_AT_REAR_PARAMS);
                fmter %platoon_size;
                fmter %current_speed_;
                fmter %current_downtrack_;
                fmter %pose_ecef_point_.ecef_x;
                fmter %pose_ecef_point_.ecef_y;
                fmter %pose_ecef_point_.ecef_z;
                
                request.strategy_params = fmter.str();
                request.urgency = 50;

                pm_.current_plan = PlatoonPlan(true, request.header.timestamp, request.header.plan_id, request.header.sender_id);
                mobility_request_publisher_(request);
                ROS_DEBUG_STREAM("Publishing request to leader " << senderId << " with params " << request.strategy_params << " and plan id = " << request.header.plan_id);
                potentialNewPlatoonId = platoonId;
            }
            else 
            {
                ROS_DEBUG_STREAM("Ignore platoon with platoon id: " << platoonId << " because it is not right in front of us");
            }
        }
        else if(isPlatoonStatusMsg) 
        {
            // If it is platoon status message, the params string is in format: STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx
            std::string statusParams = strategyParams.substr(OPERATION_STATUS_TYPE.length() + 1);
            ROS_DEBUG_STREAM("Receive operation status message from vehicle: " << senderId << " with params: " << statusParams);

            std::vector<std::string> inputsParams;
            boost::algorithm::split(inputsParams, strategyParams, boost::is_any_of(","));

            std::vector<std::string> ecef_x_parsed;
            boost::algorithm::split(ecef_x_parsed, inputsParams[3], boost::is_any_of(":"));
            double ecef_x = std::stod(ecef_x_parsed[1]);
            ROS_DEBUG_STREAM("ecef_x_parsed: " << ecef_x);

            std::vector<std::string> ecef_y_parsed;
            boost::algorithm::split(ecef_y_parsed, inputsParams[4], boost::is_any_of(":"));
            double ecef_y = std::stod(ecef_y_parsed[1]);
            ROS_DEBUG_STREAM("ecef_y_parsed: " << ecef_y);

            std::vector<std::string> ecef_z_parsed;
            boost::algorithm::split(ecef_z_parsed, inputsParams[5], boost::is_any_of(":"));
            double ecef_z = std::stod(ecef_z_parsed[1]);
            ROS_DEBUG_STREAM("ecef_z_parsed: " << ecef_z);
            
            cav_msgs::LocationECEF ecef_loc;
            ecef_loc.ecef_x = ecef_x;
            ecef_loc.ecef_y = ecef_y;
            ecef_loc.ecef_z = ecef_z;
            
            lanelet::BasicPoint2d incoming_pose = ecef_to_map_point(ecef_loc);
            
            double dtd = wm_->routeTrackPos(incoming_pose).downtrack;
            ROS_DEBUG_STREAM("dtd from ecef: " << dtd);
            pm_.memberUpdates(senderId, platoonId, msg.header.sender_bsm_id, statusParams, dtd);

        }
        else
        {
            ROS_DEBUG_STREAM("Receive operation message but ignore it because isPlatoonInfoMsg = " << isPlatoonInfoMsg << 
            ", isNotInNegotiation = " << isNotInNegotiation << " and isPlatoonStatusMsg = " << isPlatoonStatusMsg);
        }
    }

    bool PlatoonStrategicPlugin::isVehicleRightInFront(std::string rearVehicleBsmId, double downtrack) {
        double currentDtd = current_downtrack_;
        if(downtrack > currentDtd) {
            ROS_DEBUG_STREAM("Found a platoon in front. We are able to join");
            return true;
        } else {
            ROS_DEBUG_STREAM("Ignoring platoon from our back.");
            ROS_DEBUG_STREAM("The front platoon dtd is " << downtrack << " and we are current at " << currentDtd);
            return false;
        }
    }

    cav_msgs::LocationECEF PlatoonStrategicPlugin::pose_to_ecef(geometry_msgs::PoseStamped pose_msg)
    {

        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        
        cav_msgs::LocationECEF location;

        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({pose_msg.pose.position.x, pose_msg.pose.position.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;    
        

        ROS_DEBUG_STREAM("location.ecef_x: " << location.ecef_x);
        ROS_DEBUG_STREAM("location.ecef_y: " << location.ecef_y);
        ROS_DEBUG_STREAM("location.ecef_z: " << location.ecef_z);

        return location;
    }

    lanelet::BasicPoint2d PlatoonStrategicPlugin::ecef_to_map_point(cav_msgs::LocationECEF ecef_point)
    {
        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }

        lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { (double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 } , -1);

        lanelet::BasicPoint2d output {map_point.x(), map_point.y()};
        
        ROS_DEBUG_STREAM("map_point.x(): " << map_point.x());
        ROS_DEBUG_STREAM("map_point.y(): " << map_point.y());
        return output;
    } 

    void PlatoonStrategicPlugin::georeference_cb(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
    }


    cav_msgs::MobilityOperation PlatoonStrategicPlugin::composeMobilityOperationLeader(const std::string& type){
        cav_msgs::MobilityOperation msg;
        msg.header.plan_id = pm_.currentPlatoonID;
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = host_bsm_id_;
        std::string hostStaticId = config_.vehicle_id;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec()/1000000;;
        msg.strategy = MOBILITY_STRATEGY;

        if (type == OPERATION_INFO_TYPE){
            // For INFO params, the string format is INFO|REAR:%s,LENGTH:%.2f,SPEED:%.2f,SIZE:%d,DTD:%.2f

            std::string PlatoonRearBsmId = host_bsm_id_;
            int CurrentPlatoonLength = pm_.getCurrentPlatoonLength();
            double current_speed = current_speed_;
            int PlatoonSize = pm_.getTotalPlatooningSize();
            double PlatoonRearDowntrackDistance = pm_.getPlatoonRearDowntrackDistance();

            boost::format fmter(OPERATION_INFO_PARAMS);
            fmter %PlatoonRearBsmId;
            fmter %CurrentPlatoonLength;
            fmter %current_speed;
            fmter %PlatoonSize;
            fmter %PlatoonRearDowntrackDistance;
            fmter %pose_ecef_point_.ecef_x;
            fmter %pose_ecef_point_.ecef_y;
            fmter %pose_ecef_point_.ecef_z;

            std::string infoParams = fmter.str();
            msg.strategy_params = infoParams;
        }
        else if (type == OPERATION_STATUS_TYPE){
            // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
            double cmdSpeed = cmd_speed_;
            boost::format fmter(OPERATION_STATUS_PARAMS);
            fmter %cmdSpeed;
            fmter %current_downtrack_;
            fmter %current_speed_;
            fmter %pose_ecef_point_.ecef_x;
            fmter %pose_ecef_point_.ecef_y;
            fmter %pose_ecef_point_.ecef_z;
                    
            std::string statusParams = fmter.str();
            msg.strategy_params = statusParams;
        } else {
            ROS_ERROR("UNKNOW strategy param string!!!");
            msg.strategy_params = "";
        }
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);

        return msg;
    }

    cav_msgs::MobilityOperation PlatoonStrategicPlugin::composeMobilityOperationFollower()
    {
        cav_msgs::MobilityOperation msg;
        msg.header.plan_id = pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = host_bsm_id_;
        std::string hostStaticId = config_.vehicle_id;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec()/1000000;
        msg.strategy = MOBILITY_STRATEGY;
        
        double cmdSpeed = cmd_speed_;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack_;
        fmter %current_speed_;
        fmter %pose_ecef_point_.ecef_x;
        fmter %pose_ecef_point_.ecef_y;
        fmter %pose_ecef_point_.ecef_z;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);
        return msg;
    }


    cav_msgs::MobilityOperation PlatoonStrategicPlugin::composeMobilityOperationLeaderWaiting()
    {
        cav_msgs::MobilityOperation msg;
        msg.header.plan_id = pm_.currentPlatoonID;
        // This message is for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = host_bsm_id_;
        std::string hostStaticId = config_.vehicle_id;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec()/1000000;

        msg.strategy = MOBILITY_STRATEGY;
        // For STATUS params, the string format is "STATUS|CMDSPEED:5.0,DOWNTRACK:100.0,SPEED:5.0"

        double cmdSpeed = cmd_speed_;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack_;
        fmter %current_speed_;
        fmter %pose_ecef_point_.ecef_x;
        fmter %pose_ecef_point_.ecef_y;
        fmter %pose_ecef_point_.ecef_z;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        return msg;
    }

    void PlatoonStrategicPlugin::lookupECEFtoMapTransform()
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
        try
        {
            tf_ = tf2_buffer_.lookupTransform("earth", "map", ros::Time(0), ros::Duration(60.0)); //save to local copy of transform 20 sec timeout
            ROS_DEBUG_STREAM("got the transform");
        }
        catch (const tf2::TransformException &ex)
        {
        ROS_WARN("%s", ex.what());
        }
    }

    cav_msgs::MobilityOperation PlatoonStrategicPlugin::composeMobilityOperationCandidateFollower()
    {
        cav_msgs::MobilityOperation msg;

        msg.header.plan_id = pm_.currentPlatoonID;
        // All platoon mobility operation message is just for broadcast
        msg.header.recipient_id = "";
        msg.header.sender_bsm_id = host_bsm_id_;
        std::string hostStaticId = config_.vehicle_id;
        msg.header.sender_id = hostStaticId;
        msg.header.timestamp = ros::Time::now().toNSec()/1000000;
        msg.strategy = MOBILITY_STRATEGY;
        
        // For STATUS params, the string format is "STATUS|CMDSPEED:xx,DTD:xx,SPEED:xx"
        double cmdSpeed = cmd_speed_;
        boost::format fmter(OPERATION_STATUS_PARAMS);
        fmter %cmdSpeed;
        fmter %current_downtrack_;
        fmter %current_speed_;
        fmter %pose_ecef_point_.ecef_x;
        fmter %pose_ecef_point_.ecef_y;
        fmter %pose_ecef_point_.ecef_z;
                    
        std::string statusParams = fmter.str();
        msg.strategy_params = statusParams;
        ROS_DEBUG_STREAM("Composed a mobility operation message with params " << msg.strategy_params);

        return msg;
    }


}
