#pragma once

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
#include <string>
#include <vector>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/MobilityPath.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/PoseStamped.h>

namespace mobilitypath_publisher
{

    class MobilityPathPublication
    {

    public:

        MobilityPathPublication();

        // general starting point of this node
        void run();

        // local copy of pose
        boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_;

        cav_msgs::MobilityPath mobilityPathMessageGenerator(const cav_msgs::TrajectoryPlan& trajectory_plan, const geometry_msgs::TransformStamped& tf);
        

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        double spin_rate_;

        bool spinCallback();

        // ROS publisher
        ros::Publisher  path_pub_;

        // ROS subscribers
        ros::Subscriber traj_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber accel_sub_;
        ros::Subscriber bsm_sub_;

        // ROS publishers
        ros::Publisher mob_path_pub_;

        cav_msgs::TrajectoryPlan latest_trajectory_;
        cav_msgs::MobilityPath latest_mobility_path_;

        // TF listenser
        tf2_ros::Buffer tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

        // BSM Message
        cav_msgs::BSMCoreData bsm_core_;

        
        // initialize this node
        void initialize();

        // callbacks for the subscribers
        void currentpose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void trajectory_cb(const cav_msgs::TrajectoryPlanConstPtr& msg);
        void bsm_cb(const cav_msgs::BSMConstPtr& msg);

        // Compose Mobility Header
        cav_msgs::MobilityHeader composeMobilityHeader(uint64_t time);

        // Convert Trajectory Plan to (Mobility) Trajectory
        cav_msgs::Trajectory TrajectoryPlantoTrajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points, const geometry_msgs::TransformStamped& tf) const;

    
        // Convert Trajectory Point to ECEF Transform
        cav_msgs::LocationECEF TrajectoryPointtoECEF(const cav_msgs::TrajectoryPlanPoint& traj_point, const geometry_msgs::TransformStamped& tf) const;

        // sender's static ID which is its license plate
        std::string sender_id = "USDOT-49096";

        // recipient's static ID
        // Empty string indicates a broadcast message
        std::string recipient_id = "";

        // sender's dynamic ID which is its BSM id in hex string
        std::string sender_bsm_id = "FFFF";

        std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core){
            std::string res = "";
            for (size_t i=0; i<bsm_core.id.size(); i++){
                res+=std::to_string(bsm_core.id[i]);
            }
            return res;
        }

        
    };

}