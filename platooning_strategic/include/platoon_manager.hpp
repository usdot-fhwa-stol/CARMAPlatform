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
    struct PlatoonPlan {
        
        bool valid;
        long planStartTime;
        std::string planId;
        std::string peerId;
        // PlatoonPlan(){} ;
        PlatoonPlan():valid(false), planStartTime(0), planId(""), peerId("") {} ;
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
            // PlatoonMember(){};
            PlatoonMember(): staticId(""), bsmId(""), commandSpeed(0.0), vehicleSpeed(0.0), vehiclePosition(0.0), timestamp(0) {} 
            PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), vehiclePosition(vehiclePosition), timestamp(timestamp) {}
        };
        


    class PlatoonManager
    {
    public:

        PlatoonManager();

        // todo initialize as empty
        std::vector<PlatoonMember> platoon{};

        

        // Current vehicle pose in map
        geometry_msgs::PoseStamped pose_msg_;

        double current_downtrack_didtance_ = 0;

        // wm listener pointer and pointer to the actual wm object
        // std::shared_ptr<carma_wm::WMListener> wml_;
        // carma_wm::WorldModelConstPtr wm_;



        void memberUpdates(const std::string& senderId,const std::string& platoonId,const std::string& senderBsmId,const std::string& params, const double& DtD);

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


        int platoonSize = 2;
        std::string leaderID = "";
        std::string currentPlatoonID = "test_plan";
        bool isFollower = false;

        double current_speed_ = 0;
        double command_speed_ = 0;

        PlatoonState current_platoon_state = PlatoonState::STANDBY;

        PlatoonPlan current_plan;

        std::string targetLeaderId = "2";

        std::string HostMobilityId = "hostid";


    private:

    double maxAllowedJoinTimeGap = 15.0;
        double maxAllowedJoinGap = 90;
        int maxPlatoonSize = 10;
        double vehicleLength = 5.0;
        int infoMessageInterval = 200;

        std::string targetPlatoonId;
        std::string OPERATION_INFO_TYPE = "INFO";
        std::string OPERATION_STATUS_TYPE = "STATUS";
        std::string JOIN_AT_REAR_PARAMS = "SIZE:%1%,SPEED:%2%,DTD:%3%";
        std::string  MOBILITY_STRATEGY = "Carma/Platooning";
    




    double minGap_ = 22.0;
    double maxGap_ = 32.0;
    std::string previousFunctionalLeaderID_ = "";
    int previousFunctionalLeaderIndex_ = -1;

    double maxSpacing_ = 4.0;
    double minSpacing_ = 3.9;
    double lowerBoundary_ = 1.6;
    double upperBoundary_ = 1.7 ;

    double vehicleLength_ = 5.0;  // m

    double gapWithFront_ = 0.0;

    double downtrack_progress_ = 0;


    std::string algorithmType_ = "APF_ALGORITHM";

    bool insufficientGapWithPredecessor(double distanceToFrontVehicle);
    std::vector<double> calculateTimeHeadway(std::vector<double> downtrackDistance, std::vector<double> speed) const;
    int determineLeaderBasedOnViolation(std::vector<double> timeHeadways);

    // helper method for APF algorithm
    int findLowerBoundaryViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

    // helper method for APF algorithm
    int findMaximumSpacingViolationClosestToTheHostVehicle(std::vector<double> timeHeadways) const;

    std::vector<double> getTimeHeadwayFromIndex(std::vector<double> timeHeadways, int start) const;


    

    

    };
}