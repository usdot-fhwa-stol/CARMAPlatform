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

#ifndef __TRAJECTORY_EXECUTOR_HPP__
#define __TRAJECTORY_EXECUTOR_HPP__

#include <ros/ros.h>
#include <memory>
#include <map>
#include <string>
#include <mutex>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/GuidanceState.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <carma_utils/CARMAUtils.h>
#include <ros/callback_queue.h>

namespace trajectory_executor {
    /*!
     * \brief This method trims the first point off of a TrajectoryPlan's point
     * and returns a new message with the update.
     * 
     * \param plan The plan to modify
     * \return A new message with the copied contents minus the first point
     */
    cav_msgs::TrajectoryPlan trimPastPoints(const cav_msgs::TrajectoryPlan &plan);

    /**
     * Trajectory Executor package primary worker class
     * 
     * Handles subscribing to inbound plans from Plan Delegator component.
     * control plugin registration querying, and then coordination of execution
     * of that plan amongst multiple control plugins.
     */
    class TrajectoryExecutor {
        public:
            /*!
             * \brief Constructor for TrajectoryExecutor
             * \param traj_frequency Specifies trajectory output frequency in Hz
             */
            TrajectoryExecutor(int traj_frequency);

            /*!
             * \brief Constructor for TrajectoryExecutor. Uses default value for output tickrate.
             */
            TrajectoryExecutor();
            
            /*!
             * \brief Monitor the guidance state and set the current trajector as null_ptr 
             */
            void guidanceStateMonitor(cav_msgs::GuidanceState msg);

            /*!
             * \brief Initialize the TrajectoryExecutor instance by setting up 
             * all needed subscribers and publishers after querying control plugins
             * \return True if initialization was successful, false o.w.
             */
            bool init();

            /*!
             * \brief Begin processing of data and primary operation of TrajectoryExecutor.
             */
            void run();

        protected:
            /*!
             * \brief Helper function to query control plugin registration system
             * 
             * \return A map of control plugin name -> control plugin input topics
             *  for all discovered control plugins 
            */
            std::map<std::string, std::string> queryControlPlugins();

            /*!
             * \brief Callback to be invoked when a new trajectory plan is
             * received on our inbound plan topic.
             * 
             * \param msg The new TrajectoryPlan message
             */
            void onNewTrajectoryPlan(const cav_msgs::TrajectoryPlan& msg);

            /*!
             * \brief Monitor the guidance state and set the current trajector as null_ptr 
             */
            void guidanceStateCb(const cav_msgs::GuidanceStateConstPtr& msg);


            /*!
             * \brief Timer callback to be invoked at our output tickrate.
             * Outputs current trajectory plan to the first control plugin in
             * it's point list. If this is our second or later timestep on the
             * same trajectory, consumes the first point in the point list before
             * transmission.
             * 
             * \param te The timer event that triggered this callback
             */
            void onTrajEmitTick(const ros::TimerEvent& te);

        private:
            // Node handles to separate callback queues
            std::unique_ptr<ros::CARMANodeHandle> _private_nh;
            std::unique_ptr<ros::CARMANodeHandle> _public_nh;

            // Callback queues to ensure these threads process correctly
            ros::CallbackQueue _timer_callbacks;
            ros::CallbackQueue _msg_callbacks;

            ros::Subscriber _plan_sub; // Inbound plan subscriber
            ros::Subscriber _state_sub; // Guidance State subscriber
            std::map<std::string, ros::Publisher> _traj_publisher_map; // Outbound plan publishers

            // Trajectory plan tracking data. Synchronized on _cur_traj_mutex
            std::unique_ptr<cav_msgs::TrajectoryPlan> _cur_traj; 
            int _timesteps_since_last_traj {0};
            std::mutex _cur_traj_mutex;
            std::string default_control_plugin_;
            std::string default_control_plugin_topic_;

            // Timers and associated spin rates
            int _min_traj_publish_tickrate_hz {10};
            ros::Timer _timer;
            int _default_spin_rate;
    };
}

#endif 
