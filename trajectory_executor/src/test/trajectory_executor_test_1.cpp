/*
 * Copyright (C) 2018-2020 LEIDOS.
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

#include "test_utils.h"

/*!
 * \brief Test that the trajectory executor can receive an input plan and not immediately crash
 */
TEST_F(TrajectoryExecutorTestSuite, test_message_no_crash) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    ASSERT_FALSE(recv_sys_alert) << "Received system shutdown alert message from TrajectoryExecutor node.";
}

/*!
 * \brief Test that the trajectory executor will output the trajectory on 
 * the desired topic after receiving it on the input
 */
TEST_F(TrajectoryExecutorTestSuite, test_emit_traj) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj2();

    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ASSERT_GT(msg_count, 0) << "Failed to receive trajectory execution message from TrajectoryExecutor node.";
} 

/*!
 * \brief Test that the trajectory executor will shutdown properly after running 
 * over the end of it's current trajectory
 */
TEST_F(TrajectoryExecutorTestSuite, test_runover_shutdown) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    ASSERT_TRUE(recv_sys_alert) << "Failed to receive system shutdown alert message from TrajectoryExecutor node.";
}

/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_executor_test_1");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
