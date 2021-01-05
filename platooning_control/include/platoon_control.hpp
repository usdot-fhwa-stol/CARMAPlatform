#pragma once

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

#include <vector>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/Plugin.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>
#include <carma_utils/CARMAUtils.h>
#include "platoon_control_worker.hpp"




namespace platoon_control
{
    class PlatoonControlPlugin
    {
        public:
            
            // Default constructor for PlatoonControlPlugin class
            PlatoonControlPlugin();

            void initialize();

            // general starting point of this node
            void run();
			// Compose twist message by calculating speed and steering commands.
			geometry_msgs::TwistStamped composeTwist(const cav_msgs::TrajectoryPlanPoint& point);
			
			// local copy of pose
        	boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;

        
        private:


        	// CARMA ROS node handles
        	std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        	PlatoonControlWorker pcw_;

			double current_speed_ = 0.0;

			// callback function for pose
			void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

			// callback function for trajectory plan
        	void TrajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

			// callback function for current twist
			void currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist);

        	void publishTwist(const geometry_msgs::TwistStamped& twist) const;

        	// Plugin discovery message
        	cav_msgs::Plugin plugin_discovery_msg_;


        	// ROS Subscriber
        	ros::Subscriber trajectory_plan_sub;
			ros::Subscriber current_twist_sub_;
			ros::Subscriber pose_sub_;
        	// ROS Publisher
        	ros::Publisher twist_pub_;
        	
        	ros::Publisher plugin_discovery_pub_;

			// TODO: add communication to receive leader
			PlatoonLeaderInfo leader;




    
    };
}