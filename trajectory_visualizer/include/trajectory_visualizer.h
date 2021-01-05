#pragma once

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
#include <cav_msgs/TrajectoryPlan.h>
#include <carma_utils/CARMAUtils.h>
#include <visualization_msgs/MarkerArray.h>
namespace trajectory_visualizer {

    /**
     * TrajectoryVisualizer publishes 
     * 
    */ 
    const double MPH_TO_MS = 0.44704;
    
    class TrajectoryVisualizer
    {

    public:

        /**
         * \brief Default constructor
         */
        TrajectoryVisualizer();

        /**
         * \brief General starting point to run this node
         */
        void run();

    private:

        // public and private node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // publisher
        ros::Publisher traj_marker_pub_;
        
        // subscriber
        ros::Subscriber traj_sub_;
        
        // initialize this node before running
        void initialize();

        // callbacks
        void callbackPlanTrajectory(const cav_msgs::TrajectoryPlan& msg);

        // variables    
        double max_speed_;

        size_t prev_marker_list_size_ = 0;
        // we are not saving every trajectory history at this point
        //visualization_msgs::MarkerArray g_global_marker_array; 
    };

}


