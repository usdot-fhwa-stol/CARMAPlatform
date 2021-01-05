
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

#include "platoon_control_worker.hpp"


namespace platoon_control
{
   
	PlatoonControlWorker::PlatoonControlWorker(){}

	double PlatoonControlWorker::getLastSpeedCommand() const {
        return speedCmd_;
    }

    void PlatoonControlWorker::generateSpeed(const cav_msgs::TrajectoryPlanPoint& point) {
        
        PlatoonLeaderInfo leader = platoon_leader;
    	if(leader.staticId != "") {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG("The current leader position is " , leaderCurrentPosition);
	        double hostVehiclePosition = getCurrentDowntrackDistance(point);
	        double hostVehicleSpeed = currentSpeed;

	        ROS_DEBUG("The host vehicle speed is + " , hostVehicleSpeed , " and its position is " , hostVehiclePosition);
	        // If the host vehicle is the fifth vehicle and it is following the third vehicle, the leader index here is 2
	        // vehiclesInFront should be 2, because number of vehicles in front is 4, then numOfVehiclesGaps = VehicleInFront - leaderIndex   
	        int leaderIndex = leader.leaderIndex;////TODO: Communicate leader index in the platoon (plugin_.platoonManager.getIndexOf(leader);)
	        int numOfVehiclesGaps = leader.NumberOfVehicleInFront - leaderIndex;////TODO: Communicate behicles ahead in the platoon (plugin_.platoonManager.getNumberOfVehicleInFront() - leaderIndex;)
	        ROS_DEBUG("The host vehicle have " , numOfVehiclesGaps ," vehicles between itself and its leader (includes the leader)");
	        desiredGap_ = std::max(hostVehicleSpeed * timeHeadway * numOfVehiclesGaps, standStillHeadway * numOfVehiclesGaps);
	        ROS_DEBUG("The desired gap with the leader is " , desiredGap_);
	        // ROS_DEBUG("Based on raw radar, the current gap with the front vehicle is " , getDistanceToFrontVehicle());
	        double desiredHostPosition = leaderCurrentPosition - desiredGap_;
	        ROS_DEBUG("The desired host position and the setpoint for pid controller is " , desiredHostPosition);
	        // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
	        // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
	        // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
	        // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
	        // The command speed of leader vehicle will act as the baseline for our speed control
	        
	        controllerOutput = pid_ctrl_.calculate(desiredHostPosition, hostVehiclePosition);//; = speedController_.apply(signal).get().getData();

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        ROS_DEBUG("Adjusted Speed Cmd = " , adjSpeedCmd , "; Controller Output = " , controllerOutput
	        	, "; Leader CmdSpeed= " , leader.commandSpeed , "; Adjustment Cap " , adjustmentCap);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
	        if(enableMaxAdjustmentFilter) {
                if(adjSpeedCmd > leader.commandSpeed + adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed + adjustmentCap;
                } else if(adjSpeedCmd < leader.commandSpeed - adjustmentCap) {
                    adjSpeedCmd = leader.commandSpeed - adjustmentCap;
                }
                ROS_DEBUG("The adjusted cmd speed after max adjustment cap is " , adjSpeedCmd , " m/s");
            }
        
            // Third: we allow do not a large gap between two consecutive speed commands
            if(enableMaxAccelFilter) {
                
                double max = lastCmdSpeed + (maxAccel * (CMD_TIMESTEP / 1000.0));
                double min = lastCmdSpeed - (maxAccel * (CMD_TIMESTEP / 1000.0));
                if(adjSpeedCmd > max) {
                    adjSpeedCmd = max; 
                } else if (adjSpeedCmd < min) {
                    adjSpeedCmd = min;
                }
                lastCmdSpeed = adjSpeedCmd;
                ROS_DEBUG("The speed command after max accel cap is: " , adjSpeedCmd , " m/s");
            }
            speedCmd_ = adjSpeedCmd;
            ROS_DEBUG("A speed command is generated from command generator: " , speedCmd_ , " m/s");

        }

        else {
            // TODO if there is no leader available, we should change back to Leader State and re-join other platoon later
            ROS_DEBUG("There is no leader available");
            speedCmd_ = currentSpeed;
            pid_ctrl_.reset();
        }

        lastCmdSpeed = speedCmd_;

    }

    void PlatoonControlWorker::generateSteer(const cav_msgs::TrajectoryPlanPoint& point){
        pp_.current_pose_ = current_pose;
    	steerCmd_ = pp_.calculateSteer(point);
    }

    // TODO get the actual leader from strategic plugin
    void PlatoonControlWorker::setLeader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatoonControlWorker::setCurrentSpeed(double speed){
    	currentSpeed = speed;
    }

    double PlatoonControlWorker::getCurrentDowntrackDistance(const cav_msgs::TrajectoryPlanPoint& point) {
        
        double x_diff = (point.x-current_pose.position.x);
		double y_diff = (point.y-current_pose.position.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
    	return dist;
    }

    // TODO Get information about front vehicle from world model (if needed)
    // double PlatoonControlWorker::getDistanceToFrontVehicle(){
    // 	return dist_to_front_vehicle;
    // }




}