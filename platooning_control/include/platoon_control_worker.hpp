
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
#include "pid_controller.hpp"
#include "pure_pursuit.hpp"
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


        double getLastSpeedCommand() const;

        // Update speed commands based on the list of platoon members
        void generateSpeed(const cav_msgs::TrajectoryPlanPoint& point);
        void generateSteer(const cav_msgs::TrajectoryPlanPoint& point);

        // set platoon leader
        void setLeader(const PlatoonLeaderInfo& leader);
        void setCurrentSpeed(double speed);

        double speedCmd;
        double currentSpeed;
        double adjustmentCap = 10.0;
        double lastCmdSpeed = 0.0;


        double speedCmd_ = 0;
        double steerCmd_ = 0;

        PlatoonLeaderInfo platoon_leader;


        // platooning_desired_time_headway"
        double timeHeadway = 2.0;
        // platooning standstillheadway"
        double standStillHeadway = 12.0;

        void setCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
		{
			current_pose = msg->pose;
		}

		// geometry pose
		geometry_msgs::Pose current_pose;


    private:

        // pid controller object
        PIDController pid_ctrl_;

        // pure pursuit controller object
        PurePursuit pp_;

    	double maxAccel = 2.5; // m/s/s

    	double desiredTimeGap = 1.0; // s


        double desiredGap_ = 0.0;


        long CMD_TIMESTEP = 100;


        double getCurrentDowntrackDistance(const cav_msgs::TrajectoryPlanPoint& point);

        double dist_to_front_vehicle;

        bool enableMaxAdjustmentFilter = leaderSpeedCapEnabled;
        bool leaderSpeedCapEnabled = true;

        bool enableLocalSpeedLimitFilter = speedLimitCapEnabled;
        bool speedLimitCapEnabled = true;

        bool enableMaxAccelFilter = maxAccelCapEnabled;
        bool maxAccelCapEnabled = true;

    	
    

    

    };
}