/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "trajectory_executor/trajectory_executor.hpp"
#include <ros/ros.h>
#include <utility>
#include <cav_msgs/SystemAlert.h>
#include <exception>

namespace trajectory_executor 
{
    cav_msgs::TrajectoryPlan trimPastPoints(const cav_msgs::TrajectoryPlan &plan) {
        cav_msgs::TrajectoryPlan out(plan);
        out.trajectory_points = std::vector<cav_msgs::TrajectoryPlanPoint>();

        ros::Time current_time = ros::Time::now();

        for (int i = 0; i < plan.trajectory_points.size(); i++) {
            if (plan.trajectory_points[i].target_time > current_time) {
                out.trajectory_points.push_back(plan.trajectory_points[i]);
            }
        }

        return out;
    }

    TrajectoryExecutor::TrajectoryExecutor(int traj_frequency) :
        _timesteps_since_last_traj(0),
        _min_traj_publish_tickrate_hz(traj_frequency) { }

    TrajectoryExecutor::TrajectoryExecutor() :
        _timesteps_since_last_traj(0),
        _min_traj_publish_tickrate_hz(10) { }

    std::map<std::string, std::string> TrajectoryExecutor::queryControlPlugins()
    {
        // Hard coded stub for MVP since plugin manager won't be developed yet
        // TODO: Query plugin manager to receive actual list of plugins and their corresponding topics
        ROS_DEBUG("Executing stub behavior for plugin discovery MVP...");
        std::map<std::string, std::string> out;

        _private_nh->param<std::string>("default_control_plugin", default_control_plugin_, "NULL");
        _private_nh->param<std::string>("default_control_plugin_topic", default_control_plugin_topic_, "NULL");

        out[default_control_plugin_] = default_control_plugin_topic_;
        return out;
    }
    
    void TrajectoryExecutor::onNewTrajectoryPlan(const cav_msgs::TrajectoryPlan& msg)
    {
        std::unique_lock<std::mutex> lock(_cur_traj_mutex); // Acquire lock until end of this function scope
        ROS_DEBUG("Received new trajectory plan!");
        ROS_DEBUG_STREAM("New Trajectory plan ID: " << msg.trajectory_id);
        ROS_DEBUG_STREAM("New plan contains " << msg.trajectory_points.size() << " points");

        _cur_traj = std::unique_ptr<cav_msgs::TrajectoryPlan>(new cav_msgs::TrajectoryPlan(msg));
        _timesteps_since_last_traj = 0;
        ROS_DEBUG_STREAM("Successfully swapped trajectories!");
    }

    void TrajectoryExecutor::guidanceStateMonitor(cav_msgs::GuidanceState msg)
    {
        std::unique_lock<std::mutex> lock(_cur_traj_mutex); // Acquire lock until end of this function scope
        if(msg.state==cav_msgs::GuidanceState::INACTIVE)
        {
        	_cur_traj= nullptr;
        }

    }

    void TrajectoryExecutor::guidanceStateCb(const cav_msgs::GuidanceStateConstPtr& msg)
    {
        std::unique_lock<std::mutex> lock(_cur_traj_mutex);
        // TODO need to handle control handover once alernative planner system is finished
        if(msg->state != cav_msgs::GuidanceState::ENGAGED)
        {
        	_cur_traj= nullptr;
        }
    }

    void TrajectoryExecutor::onTrajEmitTick(const ros::TimerEvent& te)
    {
        std::unique_lock<std::mutex> lock(_cur_traj_mutex);
        ROS_DEBUG("TrajectoryExecutor tick start!");

        if (_cur_traj != nullptr) {
            if (_timesteps_since_last_traj > 0) {
                _cur_traj = std::unique_ptr<cav_msgs::TrajectoryPlan>(new cav_msgs::TrajectoryPlan(trimPastPoints(*_cur_traj)));
            }
            if (!_cur_traj->trajectory_points.empty()) {
                // Determine the relevant control plugin for the current timestep
                std::string control_plugin = _cur_traj->trajectory_points[0].controller_plugin_name;
                // if it instructed to use default control_plugin
                if (control_plugin == "default" || control_plugin =="")
                    control_plugin = default_control_plugin_;

                std::map<std::string, ros::Publisher>::iterator it = _traj_publisher_map.find(control_plugin);
                if (it != _traj_publisher_map.end()) {
                    ROS_DEBUG("Found match for control plugin %s at point %d in current trajectory!",
                        control_plugin.c_str(),
                        _timesteps_since_last_traj);
                    it->second.publish(*_cur_traj);
                } else {
                    std::ostringstream description_builder;
                    description_builder << "No match found for control plugin " 
                        << control_plugin << " at point " 
                        << _timesteps_since_last_traj << " in current trajectory!";

                    throw std::invalid_argument(description_builder.str());
                }
                _timesteps_since_last_traj++;
            } else {
                throw std::out_of_range("Ran out of trajectory data to consume!");
            }
        } else {
            ROS_DEBUG("Awaiting initial trajectory publication...");
        }
        ROS_DEBUG("TrajectoryExecutor tick completed succesfully!");
    }

    void TrajectoryExecutor::run()
    {
        ROS_DEBUG("Starting operations for TrajectoryExecutor component...");
        _timer = _private_nh->createTimer(
            ros::Duration(ros::Rate(this->_min_traj_publish_tickrate_hz)),
            &TrajectoryExecutor::onTrajEmitTick, 
            this);

        ROS_DEBUG("TrajectoryExecutor component started succesfully! Starting to spin.");

        ros::CARMANodeHandle::setSpinRate(_default_spin_rate);
        ros::CARMANodeHandle::spin();

        ros::shutdown();
    }

    bool TrajectoryExecutor::init()
    {
        ROS_DEBUG("Initializing TrajectoryExecutor node...");
    
        _public_nh = std::unique_ptr<ros::CARMANodeHandle>(new ros::CARMANodeHandle());
        _private_nh = std::unique_ptr<ros::CARMANodeHandle>(new ros::CARMANodeHandle("~"));
        ROS_DEBUG("Initialized all node handles");

        _private_nh->param("spin_rate", _default_spin_rate, 10);
        _private_nh->param("trajectory_publish_rate", _min_traj_publish_tickrate_hz, 10);

        ROS_DEBUG_STREAM("Initalized params with default_spin_rate " << _default_spin_rate 
            << " and trajectory_publish_rate " << _min_traj_publish_tickrate_hz);

        this->_plan_sub = this->_public_nh->subscribe<const cav_msgs::TrajectoryPlan&>("trajectory", 5, &TrajectoryExecutor::onNewTrajectoryPlan, this);
        this->_state_sub = this->_public_nh->subscribe<cav_msgs::GuidanceState>("state", 5, &TrajectoryExecutor::guidanceStateMonitor, this);

        this->_cur_traj = std::unique_ptr<cav_msgs::TrajectoryPlan>();
        ROS_DEBUG("Subscribed to inbound trajectory plans.");

        ROS_DEBUG("Setting up publishers for control plugin topics...");

        std::map<std::string, ros::Publisher> control_plugin_topics;
        auto discovered_control_plugins = queryControlPlugins();

        for (auto it = discovered_control_plugins.begin(); it != discovered_control_plugins.end(); it++)
        {
            ROS_DEBUG("Trajectory executor discovered control plugin %s listening on topic %s.", it->first.c_str(), it->second.c_str());
            ros::Publisher control_plugin_pub = _public_nh->advertise<cav_msgs::TrajectoryPlan>(it->second, 1000);
            control_plugin_topics.insert(std::make_pair(it->first, control_plugin_pub));
        }

        this->_traj_publisher_map = control_plugin_topics;
        ROS_DEBUG("TrajectoryExecutor component initialized succesfully!");

        return true;
    }
}
