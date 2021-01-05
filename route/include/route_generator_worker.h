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

#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include <cav_msgs/Route.h>
#include <cav_msgs/RouteEvent.h>
#include <cav_msgs/RouteState.h>
#include <cav_srvs/GetAvailableRoutes.h>
#include <cav_srvs/SetActiveRoute.h>
#include <cav_srvs/AbortActiveRoute.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/Geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <wgs84_utils/wgs84_utils.h>
#include <boost/filesystem.hpp>
#include <visualization_msgs/MarkerArray.h>


#include "route_state_worker.h"

namespace route {

    class RouteGeneratorWorker
    {

    public:

        /**
         * \brief Constructor for RouteGeneratorWorker class taking in dependencies via dependency injection
         * \param tf_buffer ROS tf tree buffer for getting latest tf between any two available frames
         */
        RouteGeneratorWorker(tf2_ros::Buffer& tf_buffer);
        
        /**
         * \brief Dependency injection for world model pointer.
         * \param wm CARMA world model object providing lanelet vector map and traffic regulations
         */
        void setWorldModelPtr(carma_wm::WorldModelConstPtr wm);

        /**
         * \brief Generate Lanelet2 route based on input destinations
         * \param start Lanelet 2D point in map frame indicates the starting point of selected route
         * \param via A vector of lanelet 2D points in map frame which contains points we want to visit along the route
         * \param end Lanelet 2D point in map frame indicates the final destination of selected route
         * \param map_pointer A constant pointer to lanelet vector map
         * \param graph_pointer A constant pointer to lanelet vector map routing graph
         */
        lanelet::Optional<lanelet::routing::Route> routing(const lanelet::BasicPoint2d start,
                                                        const std::vector<lanelet::BasicPoint2d>& via,
                                                        const lanelet::BasicPoint2d end,
                                                        const lanelet::LaneletMapConstPtr map_pointer,
                                                        const carma_wm::LaneletRoutingGraphConstPtr graph_pointer) const;

        /**
         * \brief Get_available_route service callback. Call this service will response with a list of route names for user to select
         * \param req A empty cav_srvs::GetAvailableRoutesRequest
         * \param resp A cav_srvs::GetAvailableRoutesResponse msg contains a list of empty Route messages with only route name populated
         */
        bool get_available_route_cb(cav_srvs::GetAvailableRoutesRequest &req, cav_srvs::GetAvailableRoutesResponse &resp);

        /**
         * \brief Set_active_route service callback. User can select a route to start following by calling this service
         * \param req A cav_srvs::SetActiveRouteRequest msg which contains the route name user wants to select
         * \param resp A cav_srvs::SetActiveRouteResponse msg contains error status showing if the routing successed
         */
        bool set_active_route_cb(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp);

        /**
         * \brief Abort_active_route service callback. User can call this service to abort a current following route and back to route selection stage
         * \param req A cav_srvs::AbortActiveRouteRequest msg which contains the route name user wants to stop following
         * \param resp A cav_srvs::AbortActiveRouteResponse msg contains error status showing if there is an active route
         */
        bool abort_active_route_cb(cav_srvs::AbortActiveRouteRequest &req, cav_srvs::AbortActiveRouteResponse &resp);

        /**
         * \brief Pose message callback. Use the latest pose message to determin if we left the route or reach the route destination
         * \param msg A geometry_msgs::PoseStampedConstPtr msg which contains current vehicle pose in the map frame
         */
        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

        /**
         * \brief Set method for configurable parameter
         * \param path The location of route files
         */
        void set_route_file_path(const std::string& path);

        /**
         * \brief Set method for configurable parameter
         * \param ct_max_error Maximum cross track error which can trigger left route event
         * \param dt_dest_range Minimum down track error which can trigger route complete event
         */
        void set_ctdt_param(double ct_max_error, double dt_dest_range);

        /**
         * \brief Method to pass publishers into worker class
         * \param route_event_pub Route event publisher
         * \param route_state_pub Route state publisher
         * \param route_pub Route publisher
         * \param route_marker_pub publisher
         */
        void set_publishers(ros::Publisher route_event_pub, ros::Publisher route_state_pub, ros::Publisher route_pub,ros::Publisher route_marker_pub);

        /**
         * \brief Helper function to load route points from route file and convert them from lat/lon values to cooridinates in ECEF
         * \param route_id This function will read the route file with provided route_id
         */
        std::vector<tf2::Vector3> load_route_destinations_in_ecef(const std::string& route_id) const;

        /**
         * \brief Helper function to transform points from ECEF frame to local map frame based on given transformation
         * \param ecef_points Points in ECEF frame stored as Vector3
         * \param map_in_earth Transformation from ECEF to map
         */
        std::vector<lanelet::BasicPoint2d> transform_to_map_frame(const std::vector<tf2::Vector3>& ecef_points, const tf2::Transform& map_in_earth) const;

        /**
         * \brief Helper function to generate a CARMA route message based on planned lanelet route
         * \param route Route object from lanelet2 lib routing function
         */
        cav_msgs::Route compose_route_msg(const lanelet::Optional<lanelet::routing::Route>& route);

        /**
         * \brief Spin callback which will be called frequently based on spin rate
         */
        bool spin_callback();
        /**
         * \brief compose_route_marker_msg is a function to generate route rviz markers
         * \param route Route object from lanelet2 lib routing function
         */
        visualization_msgs::MarkerArray compose_route_marker_msg(const lanelet::Optional<lanelet::routing::Route>& route);

    private:

        const double DEG_TO_RAD = 0.0174533;
        
        // route state worker
        RouteStateWorker rs_worker_;

        // directory of route files
        std::string route_file_path_;

        // a copy of TF transform tree
        tf2_ros::Buffer& tf_tree_;

        // const pointer to world model object
        carma_wm::WorldModelConstPtr world_model_;

        // route messages waiting to be updated and published
        cav_msgs::Route      route_msg_;
        cav_msgs::RouteEvent route_event_msg_;
        cav_msgs::RouteState route_state_msg_;
        visualization_msgs::MarkerArray route_marker_msg_;
        std::vector<lanelet::ConstPoint3d> points_; 

        // maximum cross track error which can trigger left route event
        double cross_track_max_;
        
        // minimum down track error which can trigger route complete event
        double down_track_target_range_;

        // current cross track and down track distance relative to the route
        double current_crosstrack_distance_, current_downtrack_distance_;

        // current lanelet down track and cross track distance
        double ll_crosstrack_distance_, ll_downtrack_distance_;
        lanelet::Id ll_id_;

        // current speed limit on current lanelet
        double speed_limit_ = 0;

        // local copy of Route publihsers
        ros::Publisher route_event_pub_, route_state_pub_, route_pub_,route_marker_pub_;

        // a bool flag indicates a new route has been generated such that a local copy of Route message should be published again
        bool new_route_msg_generated_ = false;
        bool new_route_marker_generated_ = false;

        // a queue of route event. All events in the queue will be published in order on each spin.
        std::queue<uint8_t> route_event_queue;

        // private helper function to add a new route event into event queue
        void publish_route_event(uint8_t event_type);        
    };
}

