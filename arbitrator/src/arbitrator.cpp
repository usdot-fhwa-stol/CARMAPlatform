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

#include "arbitrator.hpp"
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanManeuvers.h>
#include "arbitrator_utils.hpp"
#include <ros/ros.h>
#include <exception>
#include <cstdlib>

namespace arbitrator
{
    void Arbitrator::run()
    {
        ROS_INFO("Aribtrator started, beginning arbitrator state machine.");
        while (!ros::isShuttingDown())
        {
            ros::spinOnce();
            switch (sm_->get_state()) 
            {
                case INITIAL:
                    ROS_INFO("Aribtrator spinning in INITIAL state.");
                    initial_state();
                    break;
                case PLANNING:
                    ROS_INFO("Aribtrator spinning in PLANNING state.");
                    planning_state();
                    break;
                case WAITING:
                    ROS_INFO("Aribtrator spinning in WAITING state.");
                    waiting_state();
                    break;
                case PAUSED:
                    ROS_INFO("Aribtrator spinning in PAUSED state.");
                    paused_state();
                    break;
                default:
                    throw std::invalid_argument("State machine attempting to process an illegal state value");
            }
        }
    }
    
    void Arbitrator::guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg) 
    {
        switch (msg->state)
        {
            case cav_msgs::GuidanceState::STARTUP:
                // NO-OP
                break;
            case cav_msgs::GuidanceState::DRIVERS_READY:
                if(sm_->get_state() == ArbitratorState::PLANNING || sm_->get_state() == ArbitratorState::WAITING)
                {
                    ROS_INFO("Received notice that guidance has been restarted, pausing arbitrator.");
                    sm_->submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
                }
                break;
            case cav_msgs::GuidanceState::ACTIVE:
                // NO-OP
                break;
            case cav_msgs::GuidanceState::ENGAGED:
                ROS_INFO("Received notice that guidance has been engaged!");
                if (sm_->get_state() == ArbitratorState::INITIAL) {
                    sm_->submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
                } else if (sm_->get_state() == ArbitratorState::PAUSED) {
                    sm_->submit_event(ArbitratorEvent::ARBITRATOR_RESUMED);
                }
                break;
            case cav_msgs::GuidanceState::INACTIVE:
                ROS_INFO("Received notice that guidance has been disengaged, pausing arbitrator.");
                sm_->submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
                break;
            case cav_msgs::GuidanceState::SHUTDOWN:
                ROS_INFO("Received notice that guidance has been shutdown, shutting down arbitrator.");
                sm_->submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED);
                break;
            default:
                break;
        }
    }

    void Arbitrator::initial_state()
    {
        if(!initialized_)
        {   
            ROS_INFO("Arbitrator initializing on first initial state spin...");
            final_plan_pub_ = nh_->advertise<cav_msgs::ManeuverPlan>("final_maneuver_plan", 5);
            guidance_state_sub_ = nh_->subscribe<cav_msgs::GuidanceState>("guidance_state", 5, &Arbitrator::guidance_state_cb, this);
            initialized_ = true;
            // TODO: load plan duration from parameters file
        }
    }

    void Arbitrator::planning_state()
    {
        ROS_INFO("Aribtrator beginning planning process!");
        ros::Time planning_process_start = ros::Time::now();
        cav_msgs::ManeuverPlan plan = planning_strategy_.generate_plan();
        if (!plan.maneuvers.empty()) 
        {
            ros::Time plan_end_time = arbitrator_utils::get_plan_end_time(plan);
            ros::Time plan_start_time = arbitrator_utils::get_plan_start_time(plan);
            ros::Duration plan_duration = plan_end_time - plan_start_time;

            if (plan_duration < min_plan_duration_) 
            {
                ROS_WARN_STREAM("Arbitrator is unable to generate a plan with minimum plan duration requirement!");
            } 
            else 
            {
                ROS_INFO_STREAM("Arbitrator is publishing plan " << plan.maneuver_plan_id << " of duration " << plan_duration << " as current maneuver plan");
            }
            final_plan_pub_.publish(plan);
        }
        else
        {
            ROS_WARN("Arbitrator was unable to generate a plan!");
        }

        next_planning_process_start_ = planning_process_start + time_between_plans_;

        sm_->submit_event(ArbitratorEvent::PLANNING_COMPLETE);
    }

    void Arbitrator::waiting_state()
    {
        // Sleep in 100ms increments until our next planning cycle
        // This ensures we spin() at least a few times
        while (ros::Time::now() < next_planning_process_start_)
        {
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Arbitrator transitioning from WAITING to PLANNING state.");
        sm_->submit_event(ArbitratorEvent::PLANNING_TIMER_TRIGGER);
    }

    void Arbitrator::paused_state()
    {
        ros::Duration(0.1).sleep();
    }

    void Arbitrator::shutdown_state()
    {
        ROS_INFO_STREAM("Arbitrator shutting down...");
        ros::shutdown(); // Will stop upper level spin and shutdown node
    }
};
