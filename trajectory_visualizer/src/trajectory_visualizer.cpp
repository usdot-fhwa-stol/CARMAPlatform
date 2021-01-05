/*
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
 */

#include <ros/ros.h>
#include <limits>
#include <math.h>
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
        nh_->param<double>("max_speed", max_speed_, 25.0);
         // init publishers
        traj_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("trajectory_visualizer", 1);

        // init subscribers
        traj_sub_ = nh_->subscribe("plan_trajectory", 1, &TrajectoryVisualizer::callbackPlanTrajectory, this);
    }

    bool validateTime(const ros::Time& time) {

        int64_t sec64 = static_cast<int64_t>(floor(time.toSec()));
        if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
           return false;

        return true;
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
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "trajectory_visualizer";


        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 1;
        marker.frame_locked = true;

        size_t count = std::max(prev_marker_list_size_, msg.trajectory_points.size());

        for (size_t i = 1; i < count; i++)
        {
            marker.id = i;

            if (i >= msg.trajectory_points.size()) { // If we need to delete previous points
                marker.action = visualization_msgs::Marker::DELETE;
            }
            
            double max_speed = max_speed_ * MPH_TO_MS;
            double speed = max_speed;

            ros::Time t2 = msg.trajectory_points[i].target_time;
            ros::Time t1 = msg.trajectory_points[i - 1].target_time;

            if (validateTime(t2) && validateTime(t1) && t2 > t1 ) {
                ros::Duration dt = t2 - t1;
                
                double dx = msg.trajectory_points[i].x - msg.trajectory_points[i-1].x;
                double dy = msg.trajectory_points[i].y - msg.trajectory_points[i-1].y;
                double dist = sqrt(dx * dx + dy * dy);

                speed = dist/ dt.toSec();
            }



            // map color to the scale of the speed
            // red being the highest, green being the lowest (0ms)
            
            ROS_DEBUG_STREAM("Speed:" << speed << "ms, max_speed:" << max_speed << "ms");
            if (speed > max_speed) 
            {
                ROS_DEBUG_STREAM("Speed was big, so capped at " << max_speed << "ms");
                speed = max_speed;
            }

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;


            double ratio = speed / max_speed;
            if (ratio >= 0.75f)
            {
                marker.color.r = 1.0f;
            }
            else if (ratio >= 0.5f)
            {
                marker.color.b = 1.0f;
            }
            else if (ratio >= 0.25)
            {
                marker.color.b = 1.0f;
                marker.color.g = 1.0f;

            }
            else if (ratio >= 0.0)
            {
                marker.color.g = 1.0f;
            }

            marker.points = {};
            geometry_msgs::Point start;
            start.x = msg.trajectory_points[i-1].x;
            start.y = msg.trajectory_points[i-1].y;

            geometry_msgs::Point end;
            end.x = msg.trajectory_points[i].x;
            end.y = msg.trajectory_points[i].y;
            marker.points.push_back(start);
            marker.points.push_back(end);

            tmp_marker_array.markers.push_back(marker);
        }

        prev_marker_list_size_ = msg.trajectory_points.size();
        traj_marker_pub_.publish(tmp_marker_array);
    }
    
}

