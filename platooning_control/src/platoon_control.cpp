
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

#include "platoon_control.hpp"

namespace platoon_control
{
// @SONAR_STOP@
    PlatoonControlPlugin::PlatoonControlPlugin(){}
    

    void PlatoonControlPlugin::initialize(){

    	nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

	  	// Trajectory Plan Subscriber
		trajectory_plan_sub = nh_->subscribe<cav_msgs::TrajectoryPlan>("plan_trajectory", 1, &PlatoonControlPlugin::TrajectoryPlan_cb, this);
        
        // Current Twist Subscriber
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("localization/ekf_twist", 1, &PlatoonControlPlugin::currentTwist_cb, this);

		// Control Publisher
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_stamped", 10, true);

        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlPlugin::pose_cb, this);
		

		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "Platooning";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
        plugin_discovery_msg_.capability = "control/trajectory_control";

        ros::CARMANodeHandle::setSpinCallback([this]()
        {
            plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }

                                    
    void PlatoonControlPlugin::run(){
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void  PlatoonControlPlugin::TrajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
    	for(int i = 0; i < tp->trajectory_points.size() - 1; i++ ) {
    		
    		cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[i];

    		geometry_msgs::TwistStamped twist_msg = composeTwist(t1);

    		publishTwist(twist_msg);
    	}

    }

    void PlatoonControlPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }


    void PlatoonControlPlugin::currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist){
        current_speed_ = twist->twist.linear.x;
    }

    void PlatoonControlPlugin::publishTwist(const geometry_msgs::TwistStamped& twist) const {
    	twist_pub_.publish(twist);
    }

// @SONAR_START@
    geometry_msgs::TwistStamped PlatoonControlPlugin::composeTwist(const cav_msgs::TrajectoryPlanPoint& point){
    	geometry_msgs::TwistStamped current_twist;
        pcw_.setCurrentSpeed(current_speed_);
        pcw_.setLeader(leader);
        pcw_.setCurrentPose(pose_msg_);
    	pcw_.generateSpeed(point);
    	pcw_.generateSteer(point);
    	current_twist.twist.linear.x = pcw_.speedCmd_;
    	current_twist.twist.angular.z = pcw_.steerCmd_;
        current_twist.header.stamp = ros::Time::now();
    	return current_twist;
    }

}
