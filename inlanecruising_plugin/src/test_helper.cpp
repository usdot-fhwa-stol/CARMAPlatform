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


#include <ros/ros.h>
#include <cav_srvs/PlanTrajectory.h>
#include <functional>
#include <carma_utils/CARMAUtils.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_helper");
    ros::NodeHandle nh;

    boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
    {
        // flag = true;
        cav_msgs::TrajectoryPlan sending_plan;
        sending_plan.trajectory_id = "plugin_A";
        res.trajectory_plan = sending_plan;
        return true;
    };

    ros::ServiceServer service = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", cb);
    ROS_INFO("Ready to call service.");
    ros::spin();
    return 0;
};
