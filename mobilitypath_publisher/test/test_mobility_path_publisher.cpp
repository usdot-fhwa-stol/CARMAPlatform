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

#include "mobilitypath_publisher.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(MobilityPathPublicationTest, test1)
{
    ros::Time::init();
    cav_msgs::TrajectoryPlan plan;
    for (int i=1; i<5; i++){
        cav_msgs::TrajectoryPlanPoint point;
        point.x = i;
        point.y = i;
        point.target_time = ros::Time(0,i);
        plan.trajectory_points.push_back(point);
    }
    mobilitypath_publisher::MobilityPathPublication worker;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    worker.current_pose_.reset(new geometry_msgs::PoseStamped(pose));
    geometry_msgs::TransformStamped tf;
    tf.transform.translation.x = 1;
    tf.transform.translation.y = 2;
    tf.transform.translation.z = 3;
    auto res = worker.mobilityPathMessageGenerator(plan, tf);
    EXPECT_EQ(3, res.trajectory.offsets.size());
    EXPECT_EQ(1, res.trajectory.location.ecef_x);
    EXPECT_EQ(1, res.trajectory.offsets[0].offset_x);
    EXPECT_EQ(2, res.trajectory.location.ecef_y);
    EXPECT_EQ(4, res.trajectory.offsets[1].offset_y);
    EXPECT_EQ(0, res.trajectory.location.ecef_z);
    EXPECT_EQ(0, res.trajectory.offsets[2].offset_z);
}

TEST(MobilityPathPublicationTest, test2)
{
    ros::Time::init();
    cav_msgs::TrajectoryPlan plan;
    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1;
    point.y = 1;
    point.target_time = ros::Time(0,1);
    plan.trajectory_points.push_back(point);
    mobilitypath_publisher::MobilityPathPublication worker;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = -1.0;
    worker.current_pose_.reset(new geometry_msgs::PoseStamped(pose));
    geometry_msgs::TransformStamped tf;
    tf.transform.translation.x = 1;
    tf.transform.translation.y = 2;

    auto res = worker.mobilityPathMessageGenerator(plan, tf);
    EXPECT_EQ(0, res.trajectory.offsets.size());
    EXPECT_EQ(1, res.trajectory.location.ecef_x);
    EXPECT_EQ(2, res.trajectory.location.ecef_y);
    
}


// Run all the tests
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
