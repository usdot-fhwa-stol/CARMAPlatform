/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "platoon_manager.hpp"
#include "platoon_config.h"
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <array>


namespace platoon_strategic
{
    PlatoonManager::PlatoonManager()
    {}


    void PlatoonManager::memberUpdates(const std::string& senderId, const std::string& platoonId, const std::string& senderBsmId,const std::string& params, const double& DtD){

        std::vector<std::string> inputsParams;
        boost::algorithm::split(inputsParams, params, boost::is_any_of(","));

        std::vector<std::string> cmd_parsed;
        boost::algorithm::split(cmd_parsed, inputsParams[0], boost::is_any_of(":"));
        double cmdSpeed = std::stod(cmd_parsed[1]);
        ROS_DEBUG_STREAM("Command Speed: " << cmdSpeed);


        std::vector<std::string> dtd_parsed;
        boost::algorithm::split(dtd_parsed, inputsParams[1], boost::is_any_of(":"));
        double dtDistance = std::stod(dtd_parsed[1]);
        ROS_DEBUG_STREAM("Downtrack Distance: " << dtDistance);
        // get DtD directly instead of parsing message
        dtDistance = DtD;
        ROS_DEBUG_STREAM("Downtrack Distance ecef: " << dtDistance);

        std::vector<std::string> cur_parsed;
        boost::algorithm::split(cur_parsed, inputsParams[2], boost::is_any_of(":"));
        double curSpeed = std::stod(cur_parsed[1]);
        ROS_DEBUG_STREAM("Current Speed Speed: " << curSpeed);


        // If we are currently in a follower state:
        // 1. We will update platoon ID based on leader's STATUS
        // 2. We will update platoon members info based on platoon ID if it is in front of us 
        if(isFollower) 
        {
                    
            bool isFromLeader = (leaderID == senderId);
            
            bool needPlatoonIdChange = isFromLeader && (currentPlatoonID != platoonId);
            
            bool isVehicleInFrontOf = (dtDistance >= getCurrentDowntrackDistance());

            if(needPlatoonIdChange) {
                ROS_DEBUG_STREAM("It seems that the current leader is joining another platoon.");
                ROS_DEBUG_STREAM("So the platoon ID is changed from " << currentPlatoonID << " to " << platoonId);
                currentPlatoonID = platoonId;
                updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
            } 
            else if((currentPlatoonID == platoonId) && isVehicleInFrontOf) 
            {
                ROS_DEBUG_STREAM("This STATUS messages is from our platoon in front of us. Updating the info...");
                updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
                leaderID = (platoon.size()==0) ? HostMobilityId : platoon[0].staticId;
                ROS_DEBUG_STREAM("The first vehicle in our list is now " << leaderID);
            } 
            else
            {
                ROS_DEBUG_STREAM("This STATUS message is not from our platoon. We ignore this message with id: " << senderId);
            }
        }
        else
        {
            // If we are currently in any leader state, we only updates platoon member based on platoon ID
            if (currentPlatoonID == platoonId)
            {
                ROS_DEBUG_STREAM("This STATUS messages is from our platoon. Updating the info...");
                updatesOrAddMemberInfo(senderId, senderBsmId, cmdSpeed, dtDistance, curSpeed);
            }
        }

    }

    

    void PlatoonManager::updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed) {

        bool isExisted = false;
        // update/add this info into the list
        for (PlatoonMember& pm : platoon){
            if(pm.staticId == senderId) {
                pm.bsmId = senderBsmId;
                pm.commandSpeed = cmdSpeed;
                pm.vehiclePosition = dtDistance;
                pm.vehicleSpeed = curSpeed;
                pm.timestamp = ros::Time::now().toNSec()/1000000;
                ROS_DEBUG_STREAM("Receive and update platooning info on vehicel " << pm.staticId);
                ROS_DEBUG_STREAM("    BSM ID = "                                  << pm.bsmId);
                ROS_DEBUG_STREAM("    Speed = "                                   << pm.vehicleSpeed);
                ROS_DEBUG_STREAM("    Location = "                                << pm.vehiclePosition);
                ROS_DEBUG_STREAM("    CommandSpeed = "                            << pm.commandSpeed);
                isExisted = true;
                break;
            }
        }

        if(!isExisted) {
            long cur_t = ros::Time::now().toNSec()/1000000; // time in millisecond

            PlatoonMember newMember = PlatoonMember(senderId, senderBsmId, cmdSpeed, curSpeed, dtDistance, cur_t);
            platoon.push_back(newMember);

            std::sort(std::begin(platoon), std::end(platoon), [](const PlatoonMember &a, const PlatoonMember &b){return a.vehiclePosition < b.vehiclePosition;});

            ROS_DEBUG_STREAM("Add a new vehicle into our platoon list " << newMember.staticId);
        }
    }



    int PlatoonManager::getTotalPlatooningSize() const{
        if(isFollower) {
            return platoonSize;
        }
        return platoon.size() + 1;
    }
        
    double PlatoonManager::getPlatoonRearDowntrackDistance(){
        if(platoon.size() < 1) 
        {
            
            double dist = getCurrentDowntrackDistance();
            return dist;
        }
        return platoon[platoon.size() - 1].vehiclePosition;
    }

    PlatoonMember PlatoonManager::getLeader(){
        PlatoonMember leader;
        ROS_DEBUG_STREAM("platoon size: " << platoon.size());
        if(isFollower && platoon.size() != 0) 
        {
            ROS_DEBUG_STREAM("Leader initially set as first vehicle in platoon");
            // return the first vehicle in the platoon as default if no valid algorithm applied
            leader = platoon[0];
            if (algorithmType_ == "APF_ALGORITHM"){
                    int newLeaderIndex = allPredecessorFollowing();
                    if(newLeaderIndex < platoon.size() && newLeaderIndex >= 0) {
                        leader = platoon[newLeaderIndex];
                        ROS_DEBUG_STREAM("APF output: " << leader.staticId);
                        previousFunctionalLeaderIndex_ = newLeaderIndex;
                        previousFunctionalLeaderID_ = leader.staticId;
                    }
                    else {
                        // it might happened when the subject vehicle gets far away from the preceding vehicle so we follow the one in front
                        leader = platoon[platoon.size() - 1];
                        previousFunctionalLeaderIndex_ = platoon.size() - 1;
                        previousFunctionalLeaderID_ = leader.staticId;
                        ROS_DEBUG_STREAM("Based on the output of APF algorithm we start to follow our predecessor.");
                    }
            }
        }
        return leader;

    }

    int PlatoonManager::allPredecessorFollowing(){
        ///***** Case Zero *****///
        // If we are the second vehicle in this platoon,we will always follow the leader vehicle
        if(platoon.size() == 1) {
            ROS_DEBUG("As the second vehicle in the platoon, it will always follow the leader. Case Zero");
            return 0;
        }
        ///***** Case One *****///
        // If we do not have a leader in the previous time step, we follow the first vehicle as default 
        if(previousFunctionalLeaderID_ == "") {
            ROS_DEBUG("APF algorithm did not found a leader in previous time step. Case one");
            return 0;
        }
        // Generate an array of downtrack distance for every vehicles in this platoon including the host vehicle
        // The size of distance array is platoon.size() + 1, because the platoon list did not contain the host vehicle
        std::vector<double> downtrackDistance(platoon.size() + 1);
        for(int i = 0; i < platoon.size(); i++) {
            downtrackDistance[i] = platoon[i].vehiclePosition; 
        }

        // downtrackDistance[downtrackDistance.length - 1] = inputs.getDistanceFromRouteStart();
        double dt = getCurrentDowntrackDistance();
        downtrackDistance[downtrackDistance.size() - 1] = dt;
        
        // Generate an array of speed for every vehicles in this platoon including the host vehicle
        // The size of speed array is platoon.size() + 1, because the platoon list did not contain the host vehicle
        std::vector<double> speed(platoon.size() + 1);
        for(int i = 0; i < platoon.size(); i++) {
            speed[i] = platoon[i].vehicleSpeed;
        }
        
        double cur_speed = getCurrentSpeed();
        speed[speed.size() - 1] = cur_speed;

        ///***** Case Two *****///
        // If the distance headway between the subject vehicle and its predecessor is an issue
        // according to the "min_gap" and "max_gap" thresholds, then it should follow its predecessor
        // The following line will not throw exception because the length of downtrack array is larger than two in this case
        double timeHeadwayWithPredecessor = downtrackDistance[downtrackDistance.size() - 2] - downtrackDistance[downtrackDistance.size() - 1];
        gapWithFront_ = timeHeadwayWithPredecessor;
        if(insufficientGapWithPredecessor(timeHeadwayWithPredecessor)) {
            ROS_DEBUG("APF algorithm decides there is an issue with the gap with preceding vehicle: " , timeHeadwayWithPredecessor , ". Case Two");
            return platoon.size() - 1;
        }
        else{
            // implementation of the main part of APF algorithm
            // calculate the time headway between every consecutive pair of vehicles
            std::vector<double> timeHeadways = calculateTimeHeadway(downtrackDistance, speed);
            ROS_DEBUG("APF calculate time headways: " );
            for (const auto& value : timeHeadways)
            {
                ROS_DEBUG("APF time headways: " , value);
            }
            ROS_DEBUG("APF found the previous leader is " , previousFunctionalLeaderID_);
            // if the previous leader is the first vehicle in the platoon
            
            if(previousFunctionalLeaderIndex_ == 0) {
                ///***** Case Three *****///
                // If there is a violation, the return value is the desired leader index
                ROS_DEBUG("APF use violations on lower boundary or maximum spacing to choose leader. Case Three.");
                return determineLeaderBasedOnViolation(timeHeadways);
            }
            else{
                // if the previous leader is not the first vehicle
                // get the time headway between every consecutive pair of vehicles from indexOfPreviousLeader
                std::vector<double> partialTimeHeadways = getTimeHeadwayFromIndex(timeHeadways, previousFunctionalLeaderIndex_);
                ROS_DEBUG("APF partial time headways array:: " );
                for (const auto& value : partialTimeHeadways)
                {
                    ROS_DEBUG("APF partial time headways: " , value);
                }
                int closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(partialTimeHeadways);
                int closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(partialTimeHeadways);
                // if there are no violations anywhere between the subject vehicle and the current leader,
                // then depending on the time headways of the ENTIRE platoon, the subject vehicle may switch
                // leader further downstream. This is because the subject vehicle has determined that there are
                // no time headways between itself and the current leader which would cause the platoon to be unsafe.
                // if there are violations somewhere betweent the subject vehicle and the current leader,
                // then rather than assigning leadership further DOWNSTREAM, we must go further UPSTREAM in the following lines
                if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation == -1) {
                    // In order for the subject vehicle to assign leadership further downstream,
                    // two criteria must be satisfied: first the leading vehicle and its immediate follower must
                    // have a time headway greater than "upper_boundary." The purpose of this criteria is to
                    // introduce a hysteresis in order to eliminate the possibility of a vehicle continually switching back 
                    // and forth between two leaders because one of the time headways is hovering right around
                    // the "lower_boundary" threshold; second the leading vehicle and its predecessor must have
                    // a time headway less than "min_spacing" second. Just as with "upper_boundary", "min_spacing" exists to
                    // introduce a hysteresis where leaders are continually being switched.
                    bool condition1 = timeHeadways[previousFunctionalLeaderIndex_] > upperBoundary_;
                    bool condition2 = timeHeadways[previousFunctionalLeaderIndex_ - 1] < minSpacing_;
                    ///***** Case Four *****///
                    //we may switch leader further downstream
                    if(condition1 && condition2) {
                        ROS_DEBUG("APF found two conditions for assigning leadership further downstream are satisfied. Case Four");
                        return determineLeaderBasedOnViolation(timeHeadways);
                    } else {
                        ///***** Case Five *****///
                        // We may not switch leadership to another vehicle further downstream because some criteria are not satisfied
                        ROS_DEBUG("APF found two conditions for assigning leadership further downstream are not satisfied. Case Five.");
                        ROS_DEBUG("condition1: " , condition1 , " & condition2: " , condition2);
                        return previousFunctionalLeaderIndex_;
                    }
                } else if(closestLowerBoundaryViolation != -1 && closestMaximumSpacingViolation == -1) {
                    // The rest four cases have roughly the same logic: locate the closest violation and assign leadership accordingly
                    ///***** Case Six *****///
                    ROS_DEBUG("APF found closestLowerBoundaryViolation on partial time headways. Case Six.");
                    return previousFunctionalLeaderIndex_ - 1 + closestLowerBoundaryViolation;

                } else if(closestLowerBoundaryViolation == -1 && closestMaximumSpacingViolation != -1) {
                    ///***** Case Seven *****///
                    ROS_DEBUG("APF found closestMaximumSpacingViolation on partial time headways. Case Seven.");
                    return previousFunctionalLeaderIndex_ + closestMaximumSpacingViolation;
                } else{
                    ROS_DEBUG("APF found closestMaximumSpacingViolation and closestLowerBoundaryViolation on partial time headways.");
                    if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
                        ///***** Case Eight *****///
                        ROS_DEBUG("closest LowerBoundaryViolation is higher than closestMaximumSpacingViolation on partial time headways. Case Eight.");
                        return previousFunctionalLeaderIndex_ - 1 + closestLowerBoundaryViolation;
                    } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation) {
                        ///***** Case Nine *****///
                        ROS_DEBUG("closestMaximumSpacingViolation is higher than closestLowerBoundaryViolation on partial time headways. Case Nine.");
                        return previousFunctionalLeaderIndex_ + closestMaximumSpacingViolation;
                    } else {
                        ROS_DEBUG("APF Leader selection parameter is wrong!");
                        return 0;
                    }
                }
            }

        }
    }

    std::vector<double> PlatoonManager::getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const {
        std::vector<double> result(timeHeadways.begin() + start-1, timeHeadways.end());
        return result;
    }
    

    bool PlatoonManager::insufficientGapWithPredecessor(double distanceToFrontVehicle) {
        bool frontGapIsTooSmall = distanceToFrontVehicle < minGap_;
        bool previousLeaderIsPredecessor = (previousFunctionalLeaderID_ == platoon[platoon.size() - 1].staticId);
        bool frontGapIsNotLargeEnough = (distanceToFrontVehicle < maxGap_ && previousLeaderIsPredecessor);
        return (frontGapIsTooSmall || frontGapIsNotLargeEnough);
    }

    std::vector<double> PlatoonManager::calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const{
        std::vector<double> timeHeadways(downtrackDistance.size() - 1);
        for (int i=0; i<timeHeadways.size(); i++){
            if (speed[i+1]!=0)
            {
                timeHeadways[i] = (downtrackDistance[i] - downtrackDistance[i + 1]) / speed[i + 1];
            } else{
                timeHeadways[i] = std::numeric_limits<double>::infinity();
            }
        }
        return timeHeadways;
    }


    int PlatoonManager::determineLeaderBasedOnViolation(std::vector<double> timeHeadways){
        int closestLowerBoundaryViolation = findLowerBoundaryViolationClosestToTheHostVehicle(timeHeadways);
        int closestMaximumSpacingViolation = findMaximumSpacingViolationClosestToTheHostVehicle(timeHeadways);
        if(closestLowerBoundaryViolation > closestMaximumSpacingViolation) {
            ROS_DEBUG("APF found violation on closestLowerBoundaryViolation at " , closestLowerBoundaryViolation);
            return closestLowerBoundaryViolation;
        } else if(closestLowerBoundaryViolation < closestMaximumSpacingViolation){
            ROS_DEBUG("APF found violation on closestMaximumSpacingViolation at " , closestMaximumSpacingViolation);
            return closestMaximumSpacingViolation + 1;
        }
        else{
            ROS_DEBUG("APF found no violations on both closestLowerBoundaryViolation and closestMaximumSpacingViolation");
            return 0;
        }
    }

        // helper method for APF algorithm
    int PlatoonManager::findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const{
        for(int i = timeHeadways.size() - 1; i >= 0; i--) {
            if(timeHeadways[i] < lowerBoundary_) {
                return i;
            }
        }
        return -1;
    }
    
    // helper method for APF algorithm
    int PlatoonManager::findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const {
        for(int i = timeHeadways.size() - 1; i >= 0; i--) {
            if(timeHeadways[i] > maxSpacing_) {
                return i;
            }
        }
        return -1;
    }

    void PlatoonManager::changeFromFollowerToLeader() {
        isFollower = false;
        platoon = {};
        leaderID = HostMobilityId;
        currentPlatoonID = boost::uuids::to_string(boost::uuids::random_generator()());
        previousFunctionalLeaderID_ = "";
        previousFunctionalLeaderIndex_ = -1;
        ROS_DEBUG("The platoon manager is changed from follower state to leader state.");
    }

    void PlatoonManager::changeFromLeaderToFollower(std::string newPlatoonId) {
        isFollower = true;
        currentPlatoonID = newPlatoonId;
        platoon = {};
        ROS_DEBUG("The platoon manager is changed from leader state to follower state.");
    }

    int PlatoonManager::getNumberOfVehicleInFront() {
        if(isFollower) {
            return platoon.size();
        }
        return 0;
    }

    
     //Currently not used in implementation
    double PlatoonManager::getDistanceFromRouteStart() const{
        return current_downtrack_didtance_;
    }

    double PlatoonManager::getDistanceToFrontVehicle() {
        return gapWithFront_;
    }

    double PlatoonManager::getCurrentSpeed() const {
        return current_speed_;
    }

    double PlatoonManager::getCommandSpeed(){
        return command_speed_;
    }

    // TODO: make sure we are not skipping lanelets!!
    double PlatoonManager::getCurrentDowntrackDistance() const
    {
        double current_progress = current_downtrack_didtance_;
        return current_progress;
    }

    

    double PlatoonManager::getCurrentPlatoonLength() {
        if(platoon.size() == 0) {
            return vehicleLength_;
        } else {
            return getCurrentDowntrackDistance() - platoon[platoon.size() - 1].vehiclePosition + vehicleLength_; 
        }
    }


}
