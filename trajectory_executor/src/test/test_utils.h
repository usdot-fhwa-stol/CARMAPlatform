/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/SystemAlert.h>


/**
 * Test fixture for TrajectoryExecutor testing
 * Maintains publishers, subscribers, and message tracking for all tests.
 * State is reset between tests to ensure clean results.
 */
class TrajectoryExecutorTestSuite : public ::testing::Test 
{
    public:
        TrajectoryExecutorTestSuite() {
            _nh = ros::NodeHandle();
            traj_pub = _nh.advertise<cav_msgs::TrajectoryPlan>("trajectory", 5);
            traj_sub = _nh.subscribe<cav_msgs::TrajectoryPlan>("guidance/mpc_follower/trajectory", 100, 
            &TrajectoryExecutorTestSuite::trajEmitCallback, this);
            sys_alert_sub = _nh.subscribe<cav_msgs::SystemAlert>("system_alert", 100, 
            &TrajectoryExecutorTestSuite::sysAlertCallback, this);
        }
        ros::NodeHandle _nh;
        ros::Publisher traj_pub;
        ros::Subscriber traj_sub;
        ros::Subscriber sys_alert_sub;
        int msg_count = 0;
        int last_points = 9999;
        bool shrinking = true;
        bool recv_sys_alert = false;

        void trajEmitCallback(cav_msgs::TrajectoryPlan msg) {
            msg_count++;
            if (msg.trajectory_points.size() <= last_points && shrinking) {
                last_points = msg.trajectory_points.size();
            } else {
                shrinking = false;
            }
        }

        void sysAlertCallback(cav_msgs::SystemAlert msg) {
            recv_sys_alert = true;
        }
};

/*!
 * \brief Waits for the specified number of subscribers to exist on the topic
 * before continuing. Will wait until num_subscribers exist or until timeout_millis
 * has elapsed.
 * 
 * \param pub The publisher of the topic you are waiting for
 * \param num_subscribers The target number of subscribers to acheive
 * \param timeout_millis The maximum number of milliseconds to wait
 */
void waitForSubscribers(ros::Publisher pub, int num_subscribers, int timeout_millis) {
    int elapsed_millis = 0;
    while (pub.getNumSubscribers() < num_subscribers && elapsed_millis < timeout_millis) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        elapsed_millis += 100;
    }

    if (elapsed_millis > timeout_millis && pub.getNumSubscribers() < num_subscribers) {
        ROS_WARN_STREAM("Target number of subscribers for " << pub.getTopic() 
        << " not reached due to timeout! (" << pub.getNumSubscribers() 
        << "/" << num_subscribers << " subscribers)");
    }
}

/*!
 * \brief Build a small sample TrajectoryPlan message
 * 
 * \return A 10-point cav_msgs::TrajectoryPlan message containing sample data
 */
cav_msgs::TrajectoryPlan buildSampleTraj() {
    cav_msgs::TrajectoryPlan plan;

    plan.header.stamp = ros::Time::now();
    plan.trajectory_id = "TEST TRAJECTORY 1";

    ros::Time cur_time = ros::Time::now();
    for (int i = 0; i < 10; i++) {
        cav_msgs::TrajectoryPlanPoint p;
        p.controller_plugin_name = "mpc_follower";
        p.lane_id = "0";
        p.planner_plugin_name = "cruising";
        ros::Duration dur(i * 0.13);
        p.target_time = cur_time + dur;
        p.x = 10 * i;
        p.y = 10 * i;
        plan.trajectory_points.push_back(p);
    }

    return plan;
}
