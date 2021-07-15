#include "platoon_control_worker.hpp"


namespace platoon_control
{
   
	PlatoonControlWorker::PlatoonControlWorker()
    {
        pid_ctrl_ = PIDController();
        pp_ = PurePursuit();

    }

    void PlatoonControlWorker::updateConfigParams(PlatooningControlPluginConfig new_config)
    {
        ctrl_config = new_config;
        pid_ctrl_.config_ = new_config;
        pp_.config_ = new_config;
    }

	double PlatoonControlWorker::getLastSpeedCommand() const {
        return speedCmd_;
    }

    void PlatoonControlWorker::setCurrentPose(const geometry_msgs::PoseStamped msg)
	{
		current_pose_ = msg.pose;
	}

    void PlatoonControlWorker::generateSpeed(const cav_msgs::TrajectoryPlanPoint& point)
    {
        double speed_cmd = 0;
        
        PlatoonLeaderInfo leader = platoon_leader;
    	if(leader.staticId != "" && leader.staticId != ctrl_config.vehicle_id)
        {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG_STREAM("The current leader position is " << leaderCurrentPosition);

	        // double hostVehiclePosition = getCurrentDowntrackDistance(point);
	        // double hostVehicleSpeed = currentSpeed;

            // actual_gap_ = leaderCurrentPosition - hostVehiclePosition;
            // ROS_DEBUG_STREAM("actual gap " << actual_gap_);

	        // ROS_DEBUG_STREAM("The host vehicle speed is " << hostVehicleSpeed << " and its position is " << hostVehiclePosition);
	        // // If the host vehicle is the fifth vehicle and it is following the third vehicle, the leader index here is 2
	        // // vehiclesInFront should be 2, because number of vehicles in front is 4, then numOfVehiclesGaps = VehicleInFront - leaderIndex   
	        // int leaderIndex = leader.leaderIndex;
	        // int numOfVehiclesGaps = leader.NumberOfVehicleInFront - leaderIndex;
	        // ROS_DEBUG_STREAM("The host vehicle have " << numOfVehiclesGaps << " vehicles between itself and its leader (includes the leader)");
	        // double desiredGap = std::max(hostVehicleSpeed * ctrl_config.timeHeadway * numOfVehiclesGaps, ctrl_config.standStillHeadway * numOfVehiclesGaps);
            // desired_gap_ = desiredGap;
	        // ROS_DEBUG_STREAM("The desired gap with the leader is " << desiredGap);
	        // double desiredHostPosition = leaderCurrentPosition - desiredGap;
	        // ROS_DEBUG_STREAM("The desired host position and the setpoint for pid controller is " << desiredHostPosition);
	        // // PD controller is used to adjust the speed to maintain the distance gap between the subject vehicle and leader vehicle
	        // // Error input for PD controller is defined as the difference between leaderCurrentPosition and desiredLeaderPosition
	        // // A positive error implies that that the two vehicles are too far and a negative error implies that the two vehicles are too close
	        // // The summation of the leader vehicle command speed and the output of PD controller will be used as speed commands
	        // // The command speed of leader vehicle will act as the baseline for our speed control

            double desiredHostPosition = leaderCurrentPosition - desired_gap_;
            ROS_DEBUG_STREAM("desiredHostPosition = " << desiredHostPosition);

            double hostVehiclePosition = leaderCurrentPosition - actual_gap_;
            ROS_DEBUG_STREAM("hostVehiclePosition = " << hostVehiclePosition);
	        
	        controllerOutput = pid_ctrl_.calculate(desiredHostPosition, hostVehiclePosition);

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        ROS_DEBUG_STREAM("Adjusted Speed Cmd = " << adjSpeedCmd << "; Controller Output = " << controllerOutput
	        	<< "; Leader CmdSpeed= " << leader.commandSpeed << "; Adjustment Cap " << ctrl_config.adjustmentCap);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
	        
            speed_cmd = adjSpeedCmd;
            ROS_DEBUG_STREAM("A speed command is generated from command generator: " << speed_cmd << " m/s");

            if(enableMaxAdjustmentFilter)
            {
                if(speed_cmd > leader.commandSpeed + ctrl_config.adjustmentCap) {
                    speed_cmd = leader.commandSpeed + ctrl_config.adjustmentCap;
                } else if(speed_cmd < leader.commandSpeed - ctrl_config.adjustmentCap) {
                    speed_cmd = leader.commandSpeed - ctrl_config.adjustmentCap;
                }
                ROS_DEBUG_STREAM("The adjusted cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }

        }

        else if (leader.staticId == ctrl_config.vehicle_id)
        {
            ROS_DEBUG_STREAM("Host vehicle is the leader");
            speed_cmd = currentSpeed;

            if(enableMaxAdjustmentFilter) 
            {
                if(speed_cmd > ctrl_config.adjustmentCap)
                {
                    speed_cmd = ctrl_config.adjustmentCap;
                } 

                ROS_DEBUG_STREAM("The adjusted leader cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }   

            pid_ctrl_.reset();
        }

        else 
        {
            // If there is no leader available, the vehicle will stop. This means there is a mis-communication between platooning strategic and control plugins.
            ROS_DEBUG_STREAM("There is no leader available");
            speed_cmd = 0.0;
            pid_ctrl_.reset();
        }

        
        
        // Third: we allow do not a large gap between two consecutive speed commands
        if(enableMaxAccelFilter) {
                
                double max = lastCmdSpeed + (ctrl_config.maxAccel * (ctrl_config.CMD_TIMESTEP / 1000.0));
                double min = lastCmdSpeed - (ctrl_config.maxAccel * (ctrl_config.CMD_TIMESTEP / 1000.0));
                if(speed_cmd > max) {
                    speed_cmd = max; 
                } else if (speed_cmd < min) {
                    speed_cmd = min;
                }
                lastCmdSpeed = speed_cmd;
                ROS_DEBUG_STREAM("The speed command after max accel cap is: " << speed_cmd << " m/s");
        }

        speedCmd_ = speed_cmd;

        lastCmdSpeed = speedCmd_;

    }

    void PlatoonControlWorker::generateSteer(const cav_msgs::TrajectoryPlanPoint& point)
    {
        pp_.current_pose_ = current_pose_;
        pp_.velocity_ = currentSpeed;

        pp_.calculateSteer(point);
    	steerCmd_ = pp_.getSteeringAngle(); 
        angVelCmd_ = pp_.getAngularVelocity();
    }

    // TODO get the actual leader from strategic plugin
    void PlatoonControlWorker::setLeader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatoonControlWorker::setCurrentSpeed(double speed){
    	currentSpeed = speed;
    }




}