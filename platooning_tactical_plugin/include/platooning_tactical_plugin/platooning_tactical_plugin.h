
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#include <std_srvs/Trigger.h>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>
#include <vector>
#include <boost/assign/std/vector.hpp>

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <autoware_msgs/Lane.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <cav_srvs/PlanManeuvers.h>
#include <cav_srvs/PlanTrajectory.h>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

namespace platooning_tactical_plugin {

    /*!
    * Main class for the PlatooningTacticalPlugin interfacing.
    */

    class PlatooningTacticalPlugin {
        public:
            /*!
            * Constructor.
            */
            PlatooningTacticalPlugin();

            void run();

            // initialize this node
            void initialize();

            
            double start_speed = 0;
            double end_speed = 0;

            // create uneven trajectory from waypoints
            std::vector<cav_msgs::TrajectoryPlanPoint> create_uneven_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints);

            // get a sublist of waypoints marked by desired time span
            std::vector<autoware_msgs::Waypoint> get_waypoints_in_time_boundary(std::vector<autoware_msgs::Waypoint> waypoints, double time_span);

            // postprocess traj to add plugin names and shift time origin to the current ROS time
            std::vector<cav_msgs::TrajectoryPlanPoint> post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory);

            // local copy of pose
            boost::shared_ptr<geometry_msgs::PoseStamped const> pose_msg_;

            /*!
            * Destructor.
            */
            virtual ~PlatooningTacticalPlugin();

        private:

            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

            //! ROS subscribers.
            ros::Subscriber waypoint_subscriber_;
            ros::Subscriber pose_subscriber_;
            ros::Subscriber twist_subscriber_;

            // ros service servers
            ros::ServiceServer trajectory_srv_;

            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;
            // Array of waypoints
            std::vector<autoware_msgs::Waypoint> waypoints_list;

            void waypoints_cb(const autoware_msgs::LaneConstPtr& msg);
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
            void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);

            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;
            ros::Publisher platooning_tactical_plugin_discovery_pub_;

            // ROS params
            double trajectory_time_length_;
            double trajectory_point_spacing_;

            // current vehicle speed
            double current_speed_;

            // convert waypoints to a trajectory
            std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints);

    };
}