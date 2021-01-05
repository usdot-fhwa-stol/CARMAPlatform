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

#include "test_utils.h"

/*!
 * \brief Test that the Trajectory Executor properly shuts down if it encounters a trajectory
 * containing a reference to a control plugin which was not discovered.
 */
TEST_F(TrajectoryExecutorTestSuite, test_control_plugin_not_found) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    plan.trajectory_points[3].controller_plugin_name = "NULL";
    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ASSERT_TRUE(recv_sys_alert) << "Failed to receive system shutdown alert message from TrajectoryExecutor node.";
}

/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_executor_test_3");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
