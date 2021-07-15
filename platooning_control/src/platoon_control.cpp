
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

#include "platoon_control.hpp"

namespace platoon_control
{
// @SONAR_STOP@
    PlatoonControlPlugin::PlatoonControlPlugin()
    {
        pcw_ = PlatoonControlWorker();
    }
    

    void PlatoonControlPlugin::initialize(){

    	nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        PlatooningControlPluginConfig config;

        pnh_->param<double>("timeHeadway", config.timeHeadway, config.timeHeadway);
        pnh_->param<double>("standStillHeadway", config.standStillHeadway, config.standStillHeadway);
        pnh_->param<double>("maxAccel", config.maxAccel, config.maxAccel);
        pnh_->param<double>("Kp", config.Kp, config.Kp);
        pnh_->param<double>("Kd", config.Kd, config.Kd);
        pnh_->param<double>("Ki", config.Ki, config.Ki);
        pnh_->param<double>("max_value", config.max_value, config.max_value);
        pnh_->param<double>("min_value", config.min_value, config.min_value);
        pnh_->param<double>("dt", config.dt, config.dt);
        pnh_->param<double>("adjustmentCap", config.adjustmentCap, config.adjustmentCap);
        pnh_->param<double>("integratorMax", config.integratorMax, config.integratorMax);
        pnh_->param<double>("integratorMin", config.integratorMin, config.integratorMin);
        pnh_->param<double>("Kdd", config.Kdd, config.Kdd);
        pnh_->param<int>("CMD_TIMESTEP", config.CMD_TIMESTEP, config.CMD_TIMESTEP);
        pnh_->param<double>("lowpass_gain", config.lowpass_gain, config.lowpass_gain);
        pnh_->param<double>("lookahead_ratio", config.lookahead_ratio, config.lookahead_ratio);
        pnh_->param<double>("min_lookahead_dist", config.min_lookahead_dist, config.min_lookahead_dist);
        pnh_->getParam("/vehicle_id", config.vehicle_id);
        pnh_->getParam("/vehicle_wheel_base", config.vehicle_id);

        pcw_.updateConfigParams(config);
        config_ = config;

	  	// Trajectory Plan Subscriber
		trajectory_plan_sub = nh_->subscribe<cav_msgs::TrajectoryPlan>("PlatooningControlPlugin/plan_trajectory", 1, &PlatoonControlPlugin::trajectoryPlan_cb, this);
        
        // Current Twist Subscriber
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &PlatoonControlPlugin::currentTwist_cb, this);

        // Platoon Info Subscriber
        platoon_info_sub_ = nh_->subscribe<cav_msgs::PlatooningInfo>("platoon_info", 1, &PlatoonControlPlugin::platoonInfo_cb, this);

		// Control Publisher
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_raw", 5, true);
        ctrl_pub_ = nh_->advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 5, true);
        platoon_info_pub_ = nh_->advertise<cav_msgs::PlatooningInfo>("platooning_info", 1, true);


        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlPlugin::pose_cb, this);

		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatooningControlPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
        plugin_discovery_msg_.capability = "control/trajectory_control";


        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { plugin_discovery_pub_.publish(plugin_discovery_msg_); });
    }

                                    
    void PlatoonControlPlugin::run(){
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void  PlatoonControlPlugin::trajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
        
        cav_msgs::TrajectoryPlanPoint first_trajectory_point = tp->trajectory_points[1];
        cav_msgs::TrajectoryPlanPoint lookahead_point = getLookaheadTrajectoryPoint(*tp);

        trajectory_speed_ = getTrajectorySpeed(tp->trajectory_points);

    	
        
        generateControlSignals(first_trajectory_point, lookahead_point);


    }

    cav_msgs::TrajectoryPlanPoint PlatoonControlPlugin::getLookaheadTrajectoryPoint(cav_msgs::TrajectoryPlan trajectory_plan)
    {   
        cav_msgs::TrajectoryPlanPoint lookahead_point;

        double lookahead_dist = config_.lookahead_ratio * current_speed_;
        ROS_DEBUG_STREAM("lookahead based on speed: " << lookahead_dist);

        lookahead_dist = std::max(config_.min_lookahead_dist, lookahead_dist);
        ROS_DEBUG_STREAM("final lookahead: " << lookahead_dist);
            
        double traveled_dist = 0.0;
        bool found_point = false;

        for (size_t i = 1; i<trajectory_plan.trajectory_points.size() - 1; i++)
        {
            double dx =  pose_msg_.pose.position.x - trajectory_plan.trajectory_points[i].x;
            double dy =  pose_msg_.pose.position.y - trajectory_plan.trajectory_points[i].y;

            double dx1 = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
            double dy1 = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
            double dist1 = std::sqrt(dx1*dx1 + dy1*dy1);  
            ROS_DEBUG_STREAM("trajectory spacing: " << dist1);          

            double dist = std::sqrt(dx*dx + dy*dy);            

            traveled_dist = dist;

            if ((lookahead_dist - traveled_dist) < 1.0)
            {
                lookahead_point =  trajectory_plan.trajectory_points[i];
                found_point = true;
                ROS_DEBUG_STREAM("found lookahead point at index: " << i);
                break;
            }
        }

        if (!found_point)
        {
            lookahead_point = trajectory_plan.trajectory_points.back();
            ROS_DEBUG_STREAM("lookahead point set as the last trajectory point");
        }

        
        return lookahead_point;
    }

    void PlatoonControlPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
        pcw_.setCurrentPose(pose_msg_);
    }

    void PlatoonControlPlugin::platoonInfo_cb(const cav_msgs::PlatooningInfoConstPtr& msg)
    {
        
        platoon_leader_.staticId = msg->leader_id;
        platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
        platoon_leader_.commandSpeed = msg->leader_cmd_speed;
        // TODO: index is 0 temp to test the leader state
        platoon_leader_.NumberOfVehicleInFront = msg->host_platoon_position;
        platoon_leader_.leaderIndex = 0;

        ROS_DEBUG_STREAM("Platoon leader leader id:  " << platoon_leader_.staticId);
        ROS_DEBUG_STREAM("Platoon leader leader pose:  " << platoon_leader_.vehiclePosition);
        ROS_DEBUG_STREAM("Platoon leader leader cmd speed:  " << platoon_leader_.commandSpeed);

        cav_msgs::PlatooningInfo platooing_info_msg = *msg;
        // platooing_info_msg.desired_gap = pcw_.desired_gap_;
        // platooing_info_msg.actual_gap = pcw_.actual_gap_;
        pcw_.actual_gap_ = platooing_info_msg.actual_gap;
        pcw_.desired_gap_ = platooing_info_msg.desired_gap;

        platooing_info_msg.host_cmd_speed = pcw_.speedCmd_;
        platoon_info_pub_.publish(platooing_info_msg);
    }


    void PlatoonControlPlugin::currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist){
        current_speed_ = twist->twist.linear.x;
    }


// @SONAR_START@

    geometry_msgs::TwistStamped PlatoonControlPlugin::composeTwistCmd(double linear_vel, double angular_vel)
    {
        geometry_msgs::TwistStamped cmd_twist;
        cmd_twist.twist.linear.x = linear_vel;
        cmd_twist.twist.angular.z = angular_vel;
        cmd_twist.header.stamp = ros::Time::now();
        return cmd_twist;
    }

    autoware_msgs::ControlCommandStamped PlatoonControlPlugin::composeCtrlCmd(double linear_vel, double steering_angle)
    {
        autoware_msgs::ControlCommandStamped cmd_ctrl;
        cmd_ctrl.header.stamp = ros::Time::now();
        cmd_ctrl.cmd.linear_velocity = linear_vel;
        ROS_DEBUG_STREAM("ctrl command speed " << cmd_ctrl.cmd.linear_velocity);
        cmd_ctrl.cmd.steering_angle = steering_angle;
        ROS_DEBUG_STREAM("ctrl command steering " << cmd_ctrl.cmd.steering_angle);

        return cmd_ctrl;
    }

    void PlatoonControlPlugin::generateControlSignals(const cav_msgs::TrajectoryPlanPoint& first_trajectory_point, const cav_msgs::TrajectoryPlanPoint& lookahead_point){

        pcw_.setCurrentSpeed(trajectory_speed_);
        // pcw_.setCurrentSpeed(current_speed_);
        pcw_.setLeader(platoon_leader_);
    	pcw_.generateSpeed(first_trajectory_point);
    	pcw_.generateSteer(lookahead_point);


        geometry_msgs::TwistStamped twist_msg = composeTwistCmd(pcw_.speedCmd_, pcw_.angVelCmd_);
        twist_pub_.publish(twist_msg);

        autoware_msgs::ControlCommandStamped ctrl_msg = composeCtrlCmd(pcw_.speedCmd_, pcw_.steerCmd_);
        ctrl_pub_.publish(ctrl_msg);
    }

    // extract maximum speed of trajectory
    double PlatoonControlPlugin::getTrajectorySpeed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points)
    {   
        double trajectory_speed = 0;

        double dx1 = trajectory_points[trajectory_points.size()-1].x - trajectory_points[0].x;
        double dy1 = trajectory_points[trajectory_points.size()-1].y - trajectory_points[0].y;
        double d1 = sqrt(dx1*dx1 + dy1*dy1); 
        double t1 = (trajectory_points[trajectory_points.size()-1].target_time.toSec() - trajectory_points[0].target_time.toSec());

        double avg_speed = d1/t1;

        for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
        {            double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
            double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
            double d = sqrt(dx*dx + dy*dy); 
            double t = (trajectory_points[i + 1].target_time.toSec() - trajectory_points[i].target_time.toSec());
            double v = d/t;
            if(v > trajectory_speed)
            {
                trajectory_speed = v;
            }
        }

        ROS_DEBUG_STREAM("trajectory speed: " << trajectory_speed);
        ROS_DEBUG_STREAM("avg trajectory speed: " << avg_speed);

        return avg_speed;

    }

}
