
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

#include "platoon_control.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


TEST(PlatoonControlPluginTest, test1)
{

    platoon_control::PlatoonControlPlugin pc;
    
    ros::Time::init();

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    pc.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 2.0;
    point.target_time = ros::Time(0, 10);
    geometry_msgs::TwistStamped res_twist;
    res_twist = pc.composeTwist(point);
    EXPECT_NEAR(0.0,res_twist.twist.linear.x, 0.1);

}



