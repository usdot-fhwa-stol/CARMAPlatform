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
 * \brief Test that the Trajectory executor properly emits all sub-trajectories as it 
 * iterates through a single trajectory
 */
TEST_F(TrajectoryExecutorTestSuite, test_emit_multiple) {
    waitForSubscribers(traj_pub, 1, 500);
    cav_msgs::TrajectoryPlan plan = buildSampleTraj();

    traj_pub.publish(plan);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    ASSERT_LE(10, msg_count) << "Failed to receive whole trajectory from TrajectoryExecutor node.";
    ASSERT_TRUE(shrinking) << "Output trajectory plans were not shrunk each time step as expected.";
}

/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "trajectory_executor_test_2");

    std::thread spinner([] {while (ros::ok()) ros::spinOnce();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}
