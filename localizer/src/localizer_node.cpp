/*
 * Copyright (C) 2019 LEIDOS.
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

#include "localizer.h"

namespace localizer
{
	Localizer::Localizer() : spin_rate_(10.0), score_upper_limit_(2.0), unreliable_message_upper_limit_(3), localization_mode_(0) {}

	void Localizer::ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = msg->header.stamp;
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = "base_link";
		transformStamped.transform.translation.x = msg->pose.position.x;
		transformStamped.transform.translation.y = msg->pose.position.y;
		transformStamped.transform.translation.z = msg->pose.position.z;
		transformStamped.transform.rotation.x = msg->pose.orientation.x;
		transformStamped.transform.rotation.y = msg->pose.orientation.y;
		transformStamped.transform.rotation.z = msg->pose.orientation.z;
		transformStamped.transform.rotation.w = msg->pose.orientation.w;
		br_.sendTransform(transformStamped);
	}

	void Localizer::run()
	{
		
		nh_.reset(new ros::CARMANodeHandle());
		pnh_.reset(new ros::CARMANodeHandle("~"));
		pnh_->param<double>("spin_rate", spin_rate_, 10.0);
		pnh_->param<double>("score_upper_limit", score_upper_limit_, 2.0);
		pnh_->param<int>("unreliable_message_upper_limit", unreliable_message_upper_limit_, 3);
		pnh_->param<int>("localization_mode", localization_mode_, 0);

        
	}
}



