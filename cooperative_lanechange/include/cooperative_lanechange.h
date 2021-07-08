#pragma once
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

#include <vector>
#include <math.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/Geometry.h>
#include "smoothing/SplineI.h"
#include "smoothing/BSpline.h"
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/LaneChangeStatus.h>
#include <std_msgs/String.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <basic_autonomy/helper_functions.h>


namespace cooperative_lanechange
{
  // Helpful using declarations
  using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;
  
    class CooperativeLaneChangePlugin
    {
        public:
            /**
             * \brief General entry point to begin the operation of this class
            */
            void run();

            /**
             * \brief Service callback for trajectory planning
             * 
             * \param req The service request
             * \param resp The service response
             * 
             * \return True if success. False otherwise
             */ 
            bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp);
            /**
             * \brief Creates a vector of Trajectory Points from maneuver information in trajectory request
             * 
             * \param req The service request
             * 
             * \return vector of unobstructed lane change trajectory points
             */ 
            std::vector<cav_msgs::TrajectoryPlanPoint> plan_lanechange(cav_srvs::PlanTrajectoryRequest &req);
            /**
             * \brief Calculates distance between subject vehicle and vehicle 2
             * 
             * \param veh2_lanelet_id Current lanelet id of vehicle 2
             * \param veh2_downtrack Downtrack of vehicle 2 in its current lanelet
             * 
             * \return the distance between subject vehicle and vehicle 2
             */ 
            double find_current_gap(long veh2_lanelet_id, double veh2_downtrack, cav_msgs::VehicleState& state) const ;
            /**
             * \brief Converts a set of requested Lane Change maneuvers to point speed limit pairs. 
             * 
             * \param maneuvers The list of maneuvers to convert
             * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
             *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value.
             * \param wm Pointer to intialized world model for semantic map access
             * 
             * \return List of centerline points paired with speed limits
             */ 
            std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                double max_starting_downtrack,
                                                const carma_wm::WorldModelConstPtr& wm,const cav_msgs::VehicleState& state);
            /**
             * \brief Creates a Lanelet2 Linestring from a vector or points along the geometry 
             * \param starting_downtrack downtrack along route where maneuver starts
             * \param ending_downtrack downtrack along route where maneuver starts
             * \param wm Pointer to intialized world model for semantic map access
             * \return Points in a path from starting downtrack to ending downtrack
             */
            
            std::vector<lanelet::BasicPoint2d> create_route_geom(double starting_downtrack, int starting_lane_id, double ending_downtrack, const carma_wm::WorldModelConstPtr& wm);

            /**
             * \brief Given a start and end point, create a vector of points fit through a spline between the points (using a Spline library)
             * \param start_lanelet The lanelet from which lane change starts
             * \param end_lanelet The lanelet in which lane change ends
             * \return A linestring path from start to end fit through Spline Library
             */
             lanelet::BasicLineString2d create_lanechange_path(lanelet::ConstLanelet& start_lanelet, lanelet::ConstLanelet& end_lanelet);
            
            /**
             * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning
             * 
             * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
             *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
             * \param state The current state of the vehicle
             * 
             * \return A list of trajectory points to send to the carma planning stack
             */
             std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
            const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time, int starting_lanelet_id, double max_speed);

            /**
             * \brief Reduces the input points to only those points that fit within the provided time boundary
             * 
             * \param points The input point speed pairs to reduce
             * \param time_span The time span in seconds which the output points will fit within
             * 
             * \return The subset of points that fit within time_span
             */ 
            std::vector<PointSpeedPair> constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,double time_span);

            /**
             * \brief Returns a 2D coordinate frame which is located at p1 and oriented so p2 lies on the +X axis
             * 
             * \param p1 The origin point for the frame in the parent frame
             * \param p2 A point in the parent frame that will define the +X axis relative to p1
             * 
             * \return A 2D coordinate frame transform
             */ 
            Eigen::Isometry2d compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2) const;
            /**
             * \brief Computes a spline based on the provided points
             * 
             * \param basic_points The points to use for fitting the spline
             * 
             * \return A spline which has been fit to the provided points
             */ 
            std::unique_ptr<smoothing::SplineI>
            compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points);

            std::vector<double> apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits) const;
            /**
             * \brief Applies the provided speed limits to the provided speeds such that each element is capped at its corresponding speed limit if needed
             * 
             * \param speeds The speeds to limit
             * \param speed_limits The speed limits to apply. Must have the same size as speeds
             * 
             * \return The capped speed limits. Has the same size as speeds
             */ 
            double get_adaptive_lookahead(double velocity) const;

              /**
             * \brief Returns the speeds of points closest to the lookahead distance.
             * 
             * \param points The points in the map frame that the trajectory will follow. Units m
             * \param speeds Speeds assigned to points that trajectory will follow. Unit m/s
             * \param lookahead The lookahead distance to obtain future points' speed. Unit m
             * 
             * \return A vector of speed values shifted by the lookahead distance.
             */ 
            std::vector<double> get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead) const;
              /**
             * \brief Method combines input points, times, orientations, and an absolute start time to form a valid carma platform trajectory
             * 
             * NOTE: All input vectors must be the same size. The output vector will take this size.
             * 
             * \param points The points in the map frame that the trajectory will follow. Units m
             * \param times The times which at the vehicle should arrive at the specified points. First point should have a value of 0. Units s
             * \param yaws The orientation the vehicle should achieve at each point. Units radians
             * \param startTime The absolute start time which will be used to update the input relative times. Units s
             * 
             * \return A list of trajectory points built from the provided inputs.
             */
            std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
            const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
            ros::Time startTime)const;

            /**
             * \brief Callback to subscribed mobility response topic
             * \param msg Latest mobility response message
             */
            void mobilityresponse_cb(const cav_msgs::MobilityResponse& msg);

            /**
             * \brief Creates a mobility request message from planned trajectory and requested maneuver info
             * \param trajectory_plan A vector of lane change trajectory points
             * \param The mobility request message created from trajectory points, for publishing
             */
            cav_msgs::MobilityRequest create_mobility_request(std::vector<cav_msgs::TrajectoryPlanPoint>& trajectory_plan, cav_msgs::Maneuver& maneuver);

            /**
             * \brief Converts Trajectory Plan to (Mobility) Trajectory
             * \param traj_points vector of Trajectory Plan points to be converted to Trajectory type message
             * \return The Trajectory type message in world frame
             */
            
            cav_msgs::Trajectory trajectory_plan_to_trajectory(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points) const;
            /**
             * \brief Converts Trajectory Point to ECEF frame using map projection
             * \param traj_points A Trajectory Plan point to be converted to Trajectory type message
             * \throw std::invalid_argument If the map_projector_ member variable has not been set
             * \return The trajectory point message transformed to ecef frame
             */
            cav_msgs::LocationECEF trajectory_point_to_ecef(const cav_msgs::TrajectoryPlanPoint& traj_point) const;

            void add_maneuver_to_response(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp, std::vector<cav_msgs::TrajectoryPlanPoint>& planned_trajectory_points);
            
                          /**
             * \brief Given the curvature fit, computes the curvature at the given step along the curve
             * 
             * \param step_along_the_curve Value in double from 0.0 (curvature start) to 1.0 (curvature end) representing where to calculate the curvature
             * 
             * \param fit_curve curvature fit
             * 
             * \return Curvature (k = 1/r, 1/meter)
             */ 
            double compute_curvature_at(const cooperative_lanechange::smoothing::SplineI& fit_curve, double step_along_the_curve) const;

            // initialize this node
            void initialize();

            /**
             * \brief Callback for map projection string to define lat/lon -> map conversion
             * \brief msg The proj string defining the projection.
             */ 
            void georeference_callback(const std_msgs::StringConstPtr& msg);

            //Internal Variables used in unit testsis_lanechange_accepted_
            // Current vehicle forward speed
            double current_speed_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            //boolean which is updated if lane change request is accepted
            bool is_lanechange_accepted_ = false;

            ros::Publisher outgoing_mobility_request_;
            ros::Publisher lanechange_status_pub_;

            private:

            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

            ros::Publisher cooperative_lanechange_plugin_discovery_pub_;

            // ros service servers
            ros::ServiceServer trajectory_srv_;

            // ROS publishers and subscribers
            cav_msgs::Plugin plugin_discovery_msg_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            
            ros::Subscriber incoming_mobility_response_;
            ros::Subscriber bsm_sub_;
            ros::Subscriber georeference_sub_;
            ros::Timer discovery_pub_timer_;

            std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

            // trajectory frequency
            double traj_freq = 10;
            std::string DEFAULT_STRING_= "";
            //Time at which the request is first sent
            ros::Time request_sent_time;
            //boolean that records whether request has already been sent
            bool request_sent = false;
            //fraction of the maneuver completed
            double maneuver_fraction_completed_ = 0;

            // ROS params
            //Vehicle params
            std::string sender_id_ = DEFAULT_STRING_;
            cav_msgs::BSMCoreData bsm_core_;

            //Plugin specific params
            double desired_time_gap_ = 3.0;
            double trajectory_time_length_ = 6;
            std::string control_plugin_name_ = "pure_pursuit";
            double minimum_speed_ = 2.0;
            double max_accel_ = 1.5;
            double minimum_lookahead_distance_ = 5.0;
            double maximum_lookahead_distance_ = 25.0;
            double minimum_lookahead_speed_ = 2.8;
            double maximum_lookahead_speed_ =13.9;
            double lateral_accel_limit_ = 1.5;
            double speed_moving_average_window_size_ = 5;
            double curvature_moving_average_window_size_ = 5;
            double curvature_calc_lookahead_count_ = 1;
            int downsample_ratio_ =8;
            double destination_range_ = 5;
            double lanechange_time_out_ = 6.0;
            int num_points = traj_freq * trajectory_time_length_;
            double min_timestep_ = 0.1;
            double starting_downtrack_range_ = 5.0; //This parameter dictates how long before the start_dist, is it ok for the plugin to work if it's called early
            double starting_fraction_ = 0.2;
            double mid_fraction_ = 0.5;
            double min_desired_gap_ =5.0;
            double ending_buffer_downtrack_ = 5.0;
       
            cav_msgs::VehicleState  ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

            // generated trajectory plan
            cav_msgs::TrajectoryPlan trajectory_msg;
            
            /**
             * \brief Callback for the pose subscriber, which will store latest pose locally
             * \param msg Latest pose message
             */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
            
            /**
             * \brief Callback for the twist subscriber, which will store latest twist locally
             * \param msg Latest twist message
             */
            void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);

            /**
             * \brief Callback to reads bsm message from topic
             * \param msg The bsm message obtained from subscribed topic
             * message info is stored in class variable
             */
            void bsm_cb(const cav_msgs::BSMConstPtr& msg);

            std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core){
              std::string res = "";
              for (size_t i=0; i<bsm_core.id.size(); i++){
                  res+=std::to_string(bsm_core.id[i]);
              }
              return res;
            }

    
    };
}