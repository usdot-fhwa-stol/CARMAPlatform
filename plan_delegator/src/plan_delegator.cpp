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

#include <stdexcept>
#include <carma_wm/Geometry.h>
#include "plan_delegator.hpp"

namespace plan_delegator
{
    
    void PlanDelegator::init()
    {
        nh_ = ros::CARMANodeHandle();
        pnh_ = ros::CARMANodeHandle("~");

        pnh_.param<std::string>("planning_topic_prefix", planning_topic_prefix_, "/plugins/");        
        pnh_.param<std::string>("planning_topic_suffix", planning_topic_suffix_, "/plan_trajectory");
        pnh_.param<double>("spin_rate", spin_rate_, 10.0);
        pnh_.param<double>("trajectory_duration_threshold", max_trajectory_duration_, 6.0);

        traj_pub_ = nh_.advertise<cav_msgs::TrajectoryPlan>("plan_trajectory", 5);
        plan_sub_ = nh_.subscribe("final_maneuver_plan", 5, &PlanDelegator::maneuverPlanCallback, this);
        twist_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("current_velocity", 5,
            [this](const geometry_msgs::TwistStampedConstPtr& twist) {this->latest_twist_ = *twist;});
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("current_pose", 5,
            [this](const geometry_msgs::PoseStampedConstPtr& pose) {this->latest_pose_ = *pose;});
        guidance_state_sub_ = nh_.subscribe<cav_msgs::GuidanceState>("guidance_state", 5, &PlanDelegator::guidanceStateCallback, this);


        ros::CARMANodeHandle::setSpinCallback(std::bind(&PlanDelegator::spinCallback, this));
        ros::CARMANodeHandle::setSpinRate(spin_rate_);
    }
    
    void PlanDelegator::run() 
    {
        ros::CARMANodeHandle::spin();
    }

    void PlanDelegator::guidanceStateCallback(const cav_msgs::GuidanceStateConstPtr& msg)
    {
        guidance_engaged = (msg->state == cav_msgs::GuidanceState::ENGAGED);
    }

    void PlanDelegator::maneuverPlanCallback(const cav_msgs::ManeuverPlanConstPtr& plan)
    {
        ROS_INFO_STREAM("Received request to delegate plan ID " << plan->maneuver_plan_id);
        // do basic check to see if the input is valid
        if (isManeuverPlanValid(plan))
        {
            latest_maneuver_plan_ = *plan;
        }
        else {
            ROS_WARN_STREAM("Received empty plan, no maneuvers found in plan ID " << plan->maneuver_plan_id);
        }
    }

    ros::ServiceClient& PlanDelegator::getPlannerClientByName(const std::string& planner_name)
    {
        if(planner_name.size() == 0)
        {
            throw std::invalid_argument("Invalid trajectory planner name because it has zero length!");
        }
        if(trajectory_planners_.find(planner_name) == trajectory_planners_.end())
        {
            ROS_INFO_STREAM("Discovered new trajectory planner: " << planner_name);
            trajectory_planners_.emplace(
                planner_name, nh_.serviceClient<cav_srvs::PlanTrajectory>(planning_topic_prefix_ + planner_name + planning_topic_suffix_));
        }
        return trajectory_planners_[planner_name];
    }

    bool PlanDelegator::isManeuverPlanValid(const cav_msgs::ManeuverPlanConstPtr& maneuver_plan) const noexcept
    {
        // currently it only checks if maneuver list is empty
        return !maneuver_plan->maneuvers.empty();
    }

    bool PlanDelegator::isTrajectoryValid(const cav_msgs::TrajectoryPlan& trajectory_plan) const noexcept
    {
        // currently it only checks if trajectory contains less than 2 points
        return !(trajectory_plan.trajectory_points.size() < 2);
    }

    bool PlanDelegator::isManeuverExpired(const cav_msgs::Maneuver& maneuver, ros::Time current_time) const
    {
        return GET_MANEUVER_PROPERTY(maneuver, end_time) <= current_time;
    }

    cav_srvs::PlanTrajectory PlanDelegator::composePlanTrajectoryRequest(const cav_msgs::TrajectoryPlan& latest_trajectory_plan) const
    {
        auto plan_req = cav_srvs::PlanTrajectory{};
        plan_req.request.maneuver_plan = latest_maneuver_plan_;
        // set current vehicle state if we have NOT planned any previous trajectories
        if(latest_trajectory_plan.trajectory_points.empty())
        {
            plan_req.request.vehicle_state.longitudinal_vel = latest_twist_.twist.linear.x;
            plan_req.request.vehicle_state.X_pos_global = latest_pose_.pose.position.x;
            plan_req.request.vehicle_state.Y_pos_global = latest_pose_.pose.position.y;
            double roll, pitch, yaw;
            carma_wm::geometry::rpyFromQuaternion(latest_pose_.pose.orientation, roll, pitch, yaw);
            plan_req.request.vehicle_state.orientation = yaw;
        }
        // set vehicle state based on last two planned trajectory points
        else
        {
            cav_msgs::TrajectoryPlanPoint last_point = latest_trajectory_plan.trajectory_points.back();
            cav_msgs::TrajectoryPlanPoint second_last_point = *(latest_trajectory_plan.trajectory_points.rbegin() + 1);
            plan_req.request.vehicle_state.X_pos_global = last_point.x;
            plan_req.request.vehicle_state.Y_pos_global = last_point.y;
            auto distance_diff = std::sqrt(std::pow(last_point.x - second_last_point.x, 2) + std::pow(last_point.y - second_last_point.y, 2));
            ros::Duration time_diff = last_point.target_time - second_last_point.target_time;
            auto time_diff_sec = time_diff.toSec();
            // this assumes the vehicle does not have significant lateral velocity
            plan_req.request.vehicle_state.longitudinal_vel = distance_diff / time_diff_sec;
            // TODO develop way to set yaw value for future points
        }
        return plan_req;
    }

    bool PlanDelegator::isTrajectoryLongEnough(const cav_msgs::TrajectoryPlan& plan) const noexcept
    {
        ros::Duration time_diff = plan.trajectory_points.back().target_time - plan.trajectory_points.front().target_time;
        return time_diff.toSec() >= max_trajectory_duration_;
    }

    cav_msgs::TrajectoryPlan PlanDelegator::planTrajectory()
    {
        cav_msgs::TrajectoryPlan latest_trajectory_plan;
        if(!guidance_engaged)
        {
            ROS_INFO_STREAM("Guidance is not engaged. Plan delegator will not plan trajectory.");
            return latest_trajectory_plan;
        }
        // iterate through maneuver list to make service call
    
        for(const auto& maneuver : latest_maneuver_plan_.maneuvers)
        {
            // ignore expired maneuvers
            if(isManeuverExpired(maneuver))
            {
                continue;
            }
            // get corresponding ros service client for plan trajectory
            auto maneuver_planner = GET_MANEUVER_PROPERTY(maneuver, parameters.planning_tactical_plugin);
            auto client = getPlannerClientByName(maneuver_planner);
            // compose service request
            auto plan_req = composePlanTrajectoryRequest(latest_trajectory_plan);
            if(client.call(plan_req))
            {
                // validate trajectory before add to the plan
                if(!isTrajectoryValid(plan_req.response.trajectory_plan))
                {
                    ROS_WARN_STREAM("Found invalid trajectory with less than 2 trajectory points for " << latest_maneuver_plan_.maneuver_plan_id);
                    break;
                }
                latest_trajectory_plan.trajectory_points.insert(latest_trajectory_plan.trajectory_points.end(),
                                                                plan_req.response.trajectory_plan.trajectory_points.begin(),
                                                                plan_req.response.trajectory_plan.trajectory_points.end());
                latest_trajectory_plan.initial_longitudinal_velocity = plan_req.response.trajectory_plan.initial_longitudinal_velocity;
                if(isTrajectoryLongEnough(latest_trajectory_plan))
                {
                    ROS_INFO_STREAM("Plan Trajectory completed for " << latest_maneuver_plan_.maneuver_plan_id);
                    break;
                }
            }
            else
            {
                ROS_WARN_STREAM("Unsuccessful service call to trajectory planner:" << maneuver_planner << " for plan ID " << latest_maneuver_plan_.maneuver_plan_id);
                // if one service call fails, it should end plan immediately because it is there is no point to generate plan with empty space
                break;
            }
        }
        return latest_trajectory_plan;
    }

    bool PlanDelegator::spinCallback()
    {
        cav_msgs::TrajectoryPlan trajectory_plan = planTrajectory();
        // Check if planned trajectory is valid before send out
        if(isTrajectoryValid(trajectory_plan))
        {
            trajectory_plan.header.stamp = ros::Time::now();
            traj_pub_.publish(trajectory_plan);
        }
        else
        {
            ROS_WARN_STREAM("Planned trajectory is empty. It will not be published!");
        }
        return true;
    }
}
