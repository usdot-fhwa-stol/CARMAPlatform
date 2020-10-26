/*
 * Copyright (C) 2020 LEIDOS.
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
#include "trajectory_visualizer.h"

namespace trajectory_visualizer {

    TrajectoryVisualizer::TrajectoryVisualizer() {}
    void TrajectoryVisualizer::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void TrajectoryVisualizer::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("max_speed", max_speed_, 25.0);
         // init publishers
        traj_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("trajectory_visualizer", 1, true);

        // init subscribers
        traj_sub_ = nh_->subscribe("plan_trajectory", 1, &TrajectoryVisualizer::callbackPlanTrajectory, this);
    }

    void TrajectoryVisualizer::callbackPlanTrajectory(const cav_msgs::TrajectoryPlan& msg)
    {
        if (msg.trajectory_points.size() == 0)
        {
            ROS_WARN_STREAM("No trajectory point in plan_trajectory! Returning");
        }
        visualization_msgs::MarkerArray tmp_marker_array;
        // display by markers the velocity between each trajectory point/target time.
        visualization_msgs::Marker marker;
        marker.header = msg.header;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.frame_locked = true;

        for (auto i = 1; i < msg.trajectory_points.size(); i++)
        {
            ros::Duration dt = msg.trajectory_points[i].target_time - msg.trajectory_points[i - 1].target_time;
            
            double dx = msg.trajectory_points[i].x - msg.trajectory_points[i-1].x;
            double dy = msg.trajectory_points[i].y - msg.trajectory_points[i-1].y;
            double dist = sqrt(dx * dx + dy * dy);

            double speed = dist/ dt.toSec();

            // map color to the scale of the speed
            // red being the highest, green being the lowest (0ms)
            double max_speed = max_speed_ * MPH_TO_MS;
            
            ROS_DEBUG_STREAM("Speed:" << speed << "ms, max_speed:" << max_speed << "ms");
            if (speed > max_speed) 
            {
                ROS_DEBUG_STREAM("Speed was big, so capped at " << max_speed << "ms");
                speed = max_speed;
            }

            marker.color.r = 1.0f * speed / max_speed;
            marker.color.g = 1.0f - marker.color.r;
            marker.color.b = 0.0f;
            marker.color.a = 0.0f;

            marker.id = i;
            marker.pose.position.x =  msg.trajectory_points[i-1].x;
            marker.pose.position.y =  msg.trajectory_points[i-1].y;

            // double to string
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << speed;
            marker.text = oss.str();

            tmp_marker_array.markers.push_back(marker);
        }

        traj_marker_pub_.publish(tmp_marker_array);
    }
}
