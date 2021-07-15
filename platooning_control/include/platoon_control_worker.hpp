#pragma once

#include <ros/ros.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/PlanType.h>
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"
#include "platoon_control_config.h"
#include <boost/optional.hpp>







namespace platoon_control
{

	struct PlatoonLeaderInfo{
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
            // leader index in the platoon
            int leaderIndex;
            // Number of vehicles in front
            int NumberOfVehicleInFront;

            // PlatoonMember(std::string staticId, std::string bsmId, double commandSpeed, double vehicleSpeed, double vehiclePosition, long timestamp): staticId(staticId),
            // bsmId(bsmId), commandSpeed(commandSpeed), vehicleSpeed(vehicleSpeed), timestamp(timestamp) {}
        };


        // Leader info: platoonmember + leader index + number of vehicles in front


    class PlatoonControlWorker
    {
    public:

        
        PlatoonControlWorker();

        void updateConfigParams(PlatooningControlPluginConfig new_config);

        double getLastSpeedCommand() const;

        // Update speed commands based on the list of platoon members
        void generateSpeed(const cav_msgs::TrajectoryPlanPoint& point);
        void generateSteer(const cav_msgs::TrajectoryPlanPoint& point);

        // set platoon leader
        void setLeader(const PlatoonLeaderInfo& leader);
        void setCurrentSpeed(double speed);

        double speedCmd;
        double currentSpeed;
        double lastCmdSpeed = 0.0;


        double speedCmd_ = 0;
        double steerCmd_ = 0;
        double angVelCmd_ = 0;
        double desired_gap_ = ctrl_config.standStillHeadway;
        double actual_gap_ = 0.0;

        PlatoonLeaderInfo platoon_leader;


        void setCurrentPose(const geometry_msgs::PoseStamped msg);

		// geometry pose
        geometry_msgs::Pose current_pose_;


    private:
        // config parameters
        PlatooningControlPluginConfig ctrl_config;

        // pid controller object
        PIDController pid_ctrl_;

        // pure pursuit controller object
        PurePursuit pp_;

        double dist_to_front_vehicle;

        bool leaderSpeedCapEnabled = true;
        bool enableMaxAdjustmentFilter = true;
        
        bool speedLimitCapEnabled = true;
        bool enableLocalSpeedLimitFilter = true;
        
        bool maxAccelCapEnabled = true;
        bool enableMaxAccelFilter = true;
        

    };
}