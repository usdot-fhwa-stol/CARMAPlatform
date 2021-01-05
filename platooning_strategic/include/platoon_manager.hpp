
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
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/PlanType.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <autoware_msgs/ControlCommandStamped.h>




namespace platoon_strategic
{
        struct PlatoonMember{
            // Static ID is permanent ID for each vehicle
            std::string staticId;
            // Current BSM Id for each CAV
            std::string bsmId;
            // Vehicle real time command speed in m/s
            double commandSpeed;
            // Actual vehicle speed in m/s
            double vehicleSpeed;
            // Vehicle current down track distance on the current route in m
            double vehiclePosition;
            // The local time stamp when the host vehicle update any informations of this member
            long   timestamp;
            PlatoonMember(): staticId(""), bsmId(""), commandSpeed(0.0), vehicleSpeed(0.0), timestamp(0) {} 
            PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), timestamp(timestamp) {}
        };


    class PlatoonManager
    {
    public:

        std::vector<PlatoonMember> platoon;

        PlatoonManager();

        PlatoonManager(std::shared_ptr<ros::NodeHandle> nh);

        ros::Subscriber twist_sub_;
        ros::Subscriber cmd_sub_;
        ros::Subscriber pose_sub_;

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;

        // wm listener pointer and pointer to the actual wm object
        std::shared_ptr<carma_wm::WMListener> wml_;
        carma_wm::WorldModelConstPtr wm_;



        void memberUpdates(const std::string& senderId,const std::string& platoonId,const std::string& senderBsmId,const std::string& params);

        /**
         * Given any valid platooning mobility STATUS operation parameters and sender staticId,
         * in leader state this method will add/updates the information of platoon member if it is using
         * the same platoon ID, in follower state this method will updates the vehicle information who
         * is in front of the subject vehicle or update platoon id if the leader is join another platoon
         * @param senderId sender ID for the current info
         * @param platoonId sender platoon id
         * @param senderBsmId sender BSM ID
         * @param params strategy params from STATUS message in the format of "CMDSPEED:xx,DOWNTRACK:xx,SPEED:xx"
         **/
        void updatesOrAddMemberInfo(std::string senderId, std::string senderBsmId, double cmdSpeed, double dtDistance, double curSpeed);

        int getTotalPlatooningSize() const;


        PlatoonMember getLeader();

        /**
         * This is the implementation of all predecessor following (APF) algorithm for leader
         * selection in a platoon. This function will recognize who is acting as the current leader
         * of the subject vehicle. The current leader of the subject vehicle will be any ONE of
         * the vehicles in front of it. Having a vehicle further downstream function as the leader
         * is more efficient and more stable; however, having a vehicle closer to the subject vehicle
         * function as the leader is safer. For this reason, the subject vehicle will monitor
         * all time headways between every single set of consecutive vehicles starting from itself
         * to the leader. If the time headways are within some safe thresholds then vehicles further
         * downstream may function as the leader. Otherwise, for the sake of safety, vehicles closer
         * to the subject vehicle, potentially even the predecessor, will function as the leader.
         * @return the index of the leader in the platoon list
         */

        int allPredecessorFollowing();

        void changeFromFollowerToLeader();
        void changeFromLeaderToFollower(std::string newPlatoonId);
        int getNumberOfVehicleInFront();
        double getCurrentPlatoonLength();
        double getPlatoonRearDowntrackDistance();


        double getDistanceFromRouteStart() const;
        double getDistanceToFrontVehicle();
        double getCurrentSpeed() const;
        double getCommandSpeed();
        double getCurrentDowntrackDistance() const;


        int platoonSize;
        std::string leaderID;
        std::string currentPlatoonID;
        bool isFollower;

        double current_speed_;
        double command_speed_;


    private:
    

    std::shared_ptr<ros::NodeHandle> nh_;


    double minGap = 22.0;
    double maxGap = 32.0;
    std::string previousFunctionalLeaderID = "";
    int previousFunctionalLeaderIndex = -1;

    double maxSpacing = 4.0;
    double minSpacing = 3.9;
    double lowerBoundary = 1.6;
    double upperBoundary = 1.7 ;

    double vehicleLength = 5.0;  // m

    double gapWithFront = 0.0;


    std::string algorithmType = "APF_ALGORITHM";

    bool insufficientGapWithPredecessor(double distanceToFrontVehicle);
    std::vector<double> calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const;
    int determineLeaderBasedOnViolation(std::vector<double> timeHeadways);

    // helper method for APF algorithm
    int findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

    // helper method for APF algorithm
    int findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

    std::vector<double> getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const;


    void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);
    void cmd_cd(const geometry_msgs::TwistStampedConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

    std::string HostMobilityId = "hostid";

    

    };
}