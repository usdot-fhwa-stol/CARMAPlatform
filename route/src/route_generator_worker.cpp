/*
 * Copyright (C) 2020 LEIDOS.
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
#include <limits>
#include <math.h>
#include "route_generator_worker.h"
#include <functional>
#include <lanelet2_core/utility/Utilities.h>

namespace route {
    
    void RouteGeneratorWorker::setWorldModelPtr(carma_wm::WorldModelConstPtr wm)
    {
        this->world_model_ = wm;
    }

    lanelet::Optional<lanelet::routing::Route> RouteGeneratorWorker::routing(const lanelet::BasicPoint2d start,
                                                                             const std::vector<lanelet::BasicPoint2d>& via,
                                                                             const lanelet::BasicPoint2d end,
                                                                             const lanelet::LaneletMapConstPtr map_pointer,
                                                                             const carma_wm::LaneletRoutingGraphConstPtr graph_pointer) const
    {
        // find start lanelet
        auto start_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, start, 1);
        // check if there are any lanelets in the map
        if(start_lanelet_vector.empty())
        {
            ROS_ERROR_STREAM("Found no lanelets in the map. Routing cannot be done.");
            return lanelet::Optional<lanelet::routing::Route>();
        }
        // extract starting lanelet
        auto start_lanelet = lanelet::ConstLanelet(start_lanelet_vector[0].second.constData());
        // find end lanelet
        auto end_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, end, 1);
        // extract end lanelet
        auto end_lanelet = lanelet::ConstLanelet(end_lanelet_vector[0].second.constData());
        // find all via lanelets
        lanelet::ConstLanelets via_lanelets_vector;
        for(lanelet::BasicPoint2d point : via)
        {
            auto via_lanelet_vector = lanelet::geometry::findNearest(map_pointer->laneletLayer, point, 1);
            via_lanelets_vector.push_back(lanelet::ConstLanelet(via_lanelet_vector[0].second.constData()));
        }
        // routing
        return graph_pointer->getRouteVia(start_lanelet, via_lanelets_vector, end_lanelet);
    }

    void RouteGeneratorWorker::setReroutingChecker(std::function<bool()> inputFunction)
    {
        reroutingChecker=inputFunction;
    }

    bool RouteGeneratorWorker::get_available_route_cb(cav_srvs::GetAvailableRoutesRequest& req, cav_srvs::GetAvailableRoutesResponse& resp)
    {
        boost::filesystem::path route_path_object(this->route_file_path_);
        if(boost::filesystem::exists(route_path_object))
        {   
            boost::filesystem::directory_iterator end_point;
            // read all route files in the given directory
            for(boost::filesystem::directory_iterator itr(route_path_object); itr != end_point; ++itr)
            {
                if(!boost::filesystem::is_directory(itr->status()))
                {
                    auto full_file_name = itr->path().filename().generic_string();
                    cav_msgs::Route route_msg;

                    //Include logic that sorts out invalid route files based on their ending*/
                    if(full_file_name.find(".csv") != full_file_name.npos)
                     { 
                       // assume route files ending with ".csv", before that is the actual route name
                        route_msg.route_id = full_file_name.substr(0, full_file_name.find(".csv"));
                        std::ifstream fin(itr->path().generic_string());
                        std::string dest_name;
                        if(fin.is_open())
                        {
                            while (!fin.eof())
                            {
                                std::string temp;
                                std::getline(fin, temp);
                                if(temp != "") dest_name = temp;
                            }
                            fin.close();
                        } 
                        else
                        {
                           ROS_ERROR_STREAM("File open failed...");
                        }
                        auto last_comma = dest_name.find_last_of(',');
                        if(!std::isdigit(dest_name.substr(last_comma + 1).at(0)))
                        {
                            route_msg.route_name = dest_name.substr(last_comma + 1);
                            resp.availableRoutes.push_back(route_msg);
                        }
                     }
                }
            }
            
            //after route path object is available to select, worker will able to transit state and provide route selection service
            if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::LOADING) 
            {
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_LOADED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_LOADED);
            }
        }
        return true;
    }

    void RouteGeneratorWorker::set_route_file_path(const std::string& path)
    {
        this->route_file_path_ = path;
        // after route path is set, worker will able to transit state and provide route selection service
        this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_LOADED);
        publish_route_event(cav_msgs::RouteEvent::ROUTE_LOADED);
    }

    bool RouteGeneratorWorker::set_active_route_cb(cav_srvs::SetActiveRouteRequest &req, cav_srvs::SetActiveRouteResponse &resp)
    {   
        if(req.choice == cav_srvs::SetActiveRouteRequest::ROUTE_ID)
        {
            ROS_INFO_STREAM("set_active_route_cb: Selected Route ID: " << req.routeID);
        }
        else if(req.choice == cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY)
        {
            ROS_INFO_STREAM("set_active_route_cb: Destination Points array of size " << req.destination_points.size());
        }

        // only allow a new route to be activated in route selection state
        if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::SELECTION)
        {
            ROS_DEBUG_STREAM("Valid state proceeding with selection");
        	
            // entering to routing state once destinations are picked
            this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_SELECTED);
            publish_route_event(cav_msgs::RouteEvent::ROUTE_SELECTED);

            if (!vehicle_pose_) {
                ROS_ERROR_STREAM("No vehicle position. Routing cannot be completed.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            // Check if the map projection is available
            if (!map_proj_) {
                ROS_ERROR_STREAM("Could not generate route as there was no map projection available");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            // load destination points in map frame
            std::vector<lanelet::BasicPoint3d> destination_points;
            if(req.choice == cav_srvs::SetActiveRouteRequest::ROUTE_ID)
            {   
                std::vector<cav_msgs::Position3D> gps_destination_points = load_route_destination_gps_points_from_route_id(req.routeID);
                destination_points = load_route_destinations_in_map_frame(gps_destination_points);
            }
            else if(req.choice == cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY)
            {
                destination_points = load_route_destinations_in_map_frame(req.destination_points);
            }

            // Check that the requested route is valid with at least one destination point
            if(destination_points.size() < 1)
            {
                ROS_ERROR_STREAM("Provided route contains no destination points. Routing cannot be completed.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTE_FILE_ERROR;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            if (!world_model_ || !world_model_->getMap()) {
                ROS_ERROR_STREAM("World model has not been initialized.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            // convert points in 2d to map frame
            destination_points_in_map_ = lanelet::utils::transform(destination_points, [](auto a) { return lanelet::traits::to2D(a); });

            // Add vehicle as first destination point
            auto destination_points_in_map_with_vehicle = destination_points_in_map_;
            
            lanelet::BasicPoint2d vehicle_position(vehicle_pose_->pose.position.x, vehicle_pose_->pose.position.y);
            destination_points_in_map_with_vehicle.insert(destination_points_in_map_with_vehicle.begin(), vehicle_position);

            int idx = 0;
            // validate if the points are geometrically in the map
            for (auto pt : destination_points_in_map_with_vehicle)
            {
                auto llts = world_model_->getLaneletsFromPoint(pt, 1);
                if (llts.empty())
                {
                    ROS_ERROR_STREAM("Route Generator: " << idx 
                        << "th destination point is not in the map, x: " << pt.x() << " y: " << pt.y());
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTE_FILE_ERROR;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
                }
                idx ++;
            }
            
            // get route graph from world model object
            auto p = world_model_->getMapRoutingGraph();
            // generate a route
            auto route = routing(destination_points_in_map_with_vehicle.front(),
                                std::vector<lanelet::BasicPoint2d>(destination_points_in_map_with_vehicle.begin() + 1, destination_points_in_map_with_vehicle.end() - 1),
                                destination_points_in_map_with_vehicle.back(),
                                world_model_->getMap(), world_model_->getMapRoutingGraph());
            // check if route succeeded
            if(!route)
            {
                ROS_ERROR_STREAM("Cannot find a route passing all destinations.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            if (check_for_duplicate_lanelets_in_shortest_path(route.get()))
            {
                ROS_ERROR_STREAM("At least one duplicate Lanelet ID occurs in the shortest path. Routing cannot be completed.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }

            // Specify the end point of the route that is inside the last lanelet
            lanelet::Point3d end_point{lanelet::utils::getId(), destination_points_in_map_with_vehicle.back().x(), destination_points_in_map_with_vehicle.back().y(), 0};

            route->setEndPoint(end_point);

            // update route message
            route_msg_ = compose_route_msg(route);

            for(auto id : route_msg_.route_path_lanelet_ids)
            {
                auto ll = world_model_->getMap()->laneletLayer.get(id);
                route_llts.push_back(ll);

            }


            if(req.choice == cav_srvs::SetActiveRouteRequest::ROUTE_ID)
            {
                route_msg_.route_name = req.routeID;
            }
            else if(req.choice == cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY)
            {
                route_msg_.route_name = "Route";
            }
            route_marker_msg_ = compose_route_marker_msg(route);
            route_msg_.header.stamp = ros::Time::now();
            route_msg_.header.frame_id = "map";
            route_msg_.map_version = world_model_->getMapVersion();
            // since routing is done correctly, transit to route following state
            this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_STARTED);
            publish_route_event(cav_msgs::RouteEvent::ROUTE_STARTED);
            // set publish flag such that updated msg will be published in the next spin
            new_route_msg_generated_ = true;
            return true;
        }

        ROS_ERROR_STREAM("System is already following a route.");
        resp.errorStatus = cav_srvs::SetActiveRouteResponse::ALREADY_FOLLOWING_ROUTE;

        return true;
    }

    bool RouteGeneratorWorker::check_for_duplicate_lanelets_in_shortest_path(const lanelet::routing::Route& route) const
    {
        // Create a vector for the lanelet IDs in the shortest path
        std::vector<lanelet::Id> shortest_path_lanelet_ids;

        // Iterate through the shortest path to populate shortest_path_lanelet_ids with lanelet IDs
        for(const auto& ll : route.shortestPath())
        {
            shortest_path_lanelet_ids.push_back(ll.id());
        }

        // Verify that there are no duplicate lanelet IDs in the shortest path
        std::sort(shortest_path_lanelet_ids.begin(), shortest_path_lanelet_ids.end());
        auto it = std::adjacent_find(shortest_path_lanelet_ids.begin(), shortest_path_lanelet_ids.end());
        
        if (it != shortest_path_lanelet_ids.end())
        {
            // Route's shortest path contains duplicate lanelet IDs
            return true;
        }

        // Route's shortest path does not duplicate lanelet IDs
        return false;
    }

    std::vector<lanelet::BasicPoint3d> RouteGeneratorWorker::load_route_destinations_in_map_frame(const std::vector<cav_msgs::Position3D>& destinations) const
    {
        if (!map_proj_) {
            throw std::invalid_argument("load_route_destinations_in_map_frame (using destination points array) before map projection was set");
        }
        
        lanelet::projection::LocalFrameProjector projector(map_proj_.get().c_str()); // Build map projector

        // Process each point in 'destinations'
        std::vector<lanelet::BasicPoint3d> destination_points;
        for (const auto& destination : destinations)
        {
            lanelet::GPSPoint coordinate;
            coordinate.lon = destination.longitude;
            coordinate.lat = destination.latitude;
            
            if(destination.elevation_exists)
            {
                coordinate.ele = destination.elevation;
            }
            else
            {
                coordinate.ele = 0.0;
            }

            destination_points.emplace_back(projector.forward(coordinate));
        }

        return destination_points;
    }

    std::vector<cav_msgs::Position3D> RouteGeneratorWorker::load_route_destination_gps_points_from_route_id(const std::string& route_id) const
    {
        // compose full path of the route file
        std::string route_file_name = route_file_path_ + route_id + ".csv";
        std::ifstream fs(route_file_name);
        std::string line;
        
        // read each line in route file (if any)
        std::vector<cav_msgs::Position3D> destination_points;
        while(std::getline(fs, line))
        {
            cav_msgs::Position3D gps_point;

            // lat lon and elev is seperated by comma
            auto comma = line.find(",");
            // convert lon value in degrees from string
            gps_point.longitude = std::stod(line.substr(0, comma));
            line.erase(0, comma + 1);
            comma = line.find(",");
            // convert lat value in degrees from string
            gps_point.latitude = std::stod(line.substr(0, comma));
            // elevation is in meters
            line.erase(0, comma + 1);
            comma = line.find(",");
            gps_point.elevation = std::stod(line.substr(0, comma));
            gps_point.elevation_exists = true;

            destination_points.push_back(gps_point);
        }

        return destination_points;
    }

    visualization_msgs::Marker RouteGeneratorWorker::compose_route_marker_msg(const lanelet::Optional<lanelet::routing::Route>& route)
    {
        std::vector<lanelet::ConstPoint3d> points;
        auto end_point_3d = route.get().getEndPoint();
        auto last_ll = route.get().shortestPath().back();
        double end_point_downtrack = carma_wm::geometry::trackPos(last_ll, {end_point_3d.x(), end_point_3d.y()}).downtrack;
        double lanelet_downtrack = carma_wm::geometry::trackPos(last_ll, last_ll.centerline().back().basicPoint2d()).downtrack;
        // get number of points to display using ratio of the downtracks
        int points_until_end_point = (int) last_ll.centerline().size() * (end_point_downtrack / lanelet_downtrack);
  
        for(const auto& ll : route.get().shortestPath())
        {
            if (ll.id() == last_ll.id())
            {
                for (int i = 0; i < points_until_end_point; i++)
                {
                    points.push_back(ll.centerline()[i]);
                }
                continue;
            }
            for(const auto& pt : ll.centerline())
            {
                points.push_back(pt);
            }
        }

        route_marker_msg_.points={};

        // create the marker msgs
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::SPHERE_LIST;//
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "route_visualizer";

        marker.scale.x = 0.65;
        marker.scale.y = 0.65;
        marker.scale.z = 0.65;
        marker.frame_locked = true;

        marker.id = 0;
        marker.color.r = 1.0F;
        marker.color.g = 1.0F;
        marker.color.b = 1.0F;
        marker.color.a = 1.0F;

        if (points.empty())
        {
            ROS_WARN_STREAM("No central line points! Returning");
            return route_marker_msg_;
        }
 
        for (int i = 0; i < points.size(); i=i+5)
        {
            geometry_msgs::Point temp_point;
            temp_point.x = points[i].x();
            temp_point.y = points[i].y();
            temp_point.z = 1; //to show up on top of the lanelet lines
            
            marker.points.push_back(temp_point);
        }
        new_route_marker_generated_ = true;
        return marker;
    }

    cav_msgs::Route RouteGeneratorWorker::compose_route_msg(const lanelet::Optional<lanelet::routing::Route>& route)
    {
        cav_msgs::Route msg;
        // iterate through the shortest path to populate shortest_path_lanelet_ids
        for(const auto& ll : route.get().shortestPath())
        {
            msg.shortest_path_lanelet_ids.push_back(ll.id());
        }
        // iterate through all lanelet in the route to populate route_path_lanelet_ids
        for(const auto& ll : route.get().laneletSubmap()->laneletLayer)
        {
            msg.route_path_lanelet_ids.push_back(ll.id());
        }
        msg.end_point.x  = route->getEndPoint().x();
        msg.end_point.y  = route->getEndPoint().y();
        msg.end_point.z  = route->getEndPoint().z();

        return msg;
    }

    bool RouteGeneratorWorker::abort_active_route_cb(cav_srvs::AbortActiveRouteRequest &req, cav_srvs::AbortActiveRouteResponse &resp)
    {
        // only make sense to abort when it is in route following state
        if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::FOLLOWING)
        {
            this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_ABORTED);
            resp.error_status = cav_srvs::AbortActiveRouteResponse::NO_ERROR;
            publish_route_event(cav_msgs::RouteEvent::ROUTE_ABORTED);
            route_msg_ = cav_msgs::Route{};
        } else {
            // service call successed but there is not active route
            resp.error_status = cav_srvs::AbortActiveRouteResponse::NO_ACTIVE_ROUTE;
        }
        return true;
    }

    void RouteGeneratorWorker::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        vehicle_pose_ = *msg;
        if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::FOLLOWING) {
            // convert from pose stamp into lanelet basic 2D point
            current_loc_ = lanelet::BasicPoint2d(msg->pose.position.x, msg->pose.position.y);
            // get dt ct from world model
            carma_wm::TrackPos track(0.0, 0.0);
            try {
                track = this->world_model_->routeTrackPos(current_loc_);
            } catch (std::invalid_argument ex) {
                ROS_WARN_STREAM("Routing has finished but carma_wm has not receive it!");
                return;
            }
            auto current_lanelet = get_closest_lanelet_from_route_llts(current_loc_);
            auto lanelet_track = carma_wm::geometry::trackPos(current_lanelet, current_loc_);
            ll_id_ = current_lanelet.id();
            ll_crosstrack_distance_ = lanelet_track.crosstrack;
            ll_downtrack_distance_ = lanelet_track.downtrack;
            current_crosstrack_distance_ = track.crosstrack;
            current_downtrack_distance_ = track.downtrack;
            // Determine speed limit
            lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = world_model_->getTrafficRules();
            
            if (traffic_rules) 
            {
                auto laneletIterator = world_model_->getMap()->laneletLayer.find(ll_id_);
                if (laneletIterator != world_model_->getMap()->laneletLayer.end()) {
                    speed_limit_ = (*traffic_rules)->speedLimit(*laneletIterator).speedLimit.value();
                } 
                else 
                {
                    ROS_ERROR_STREAM("Failed to set the current speed limit. The lanelet_id: "
                        << ll_id_ << " could not be matched with a lanelet in the map. The previous speed limit of "
                        << speed_limit_ << " will be used.");
                }
                
            } 
            else 
            {
                ROS_ERROR_STREAM("Failed to set the current speed limit. Valid traffic rules object could not be built.");
            }
            // check if we left the seleted route by cross track error
            bool departed = crosstrack_error_check(msg, current_lanelet);
            if (departed)
                {
                    this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_DEPARTED);
                    publish_route_event(cav_msgs::RouteEvent::ROUTE_DEPARTED);
                }

            // check if we reached our destination be remaining down track distance
            auto end_point_3d = world_model_->getRoute()->getEndPoint();
            auto last_ll = world_model_->getRoute()->shortestPath().back();
            double end_point_downtrack = carma_wm::geometry::trackPos(last_ll, {end_point_3d.x(), end_point_3d.y()}).downtrack;
            double last_lanelet_downtrack = carma_wm::geometry::trackPos(last_ll, last_ll.centerline().back().basicPoint2d()).downtrack;
            
            double route_length_2d = world_model_->getRoute()->length2d() - (last_lanelet_downtrack - end_point_downtrack);
            if((current_downtrack_distance_ > route_length_2d - down_track_target_range_ && current_speed_ < epsilon_) || (current_downtrack_distance_ > route_length_2d))
            {
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_COMPLETED);
            }
        }
    }

    void RouteGeneratorWorker::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void RouteGeneratorWorker::georeference_cb(const std_msgs::StringConstPtr& msg)
    {
        map_proj_ = msg->data;
    }

    void RouteGeneratorWorker::set_publishers(ros::Publisher route_event_pub, ros::Publisher route_state_pub, ros::Publisher route_pub,ros::Publisher route_marker_pub)
    {
        route_event_pub_ = route_event_pub;
        route_state_pub_ = route_state_pub;
        route_pub_ = route_pub;
        route_marker_pub_= route_marker_pub;
    }

    void RouteGeneratorWorker::set_ctdt_param(double ct_max_error, double dt_dest_range)
    {
        this->cross_track_max_ = ct_max_error;
        this->down_track_target_range_ = dt_dest_range;
    }

    void RouteGeneratorWorker::publish_route_event(uint8_t event_type)
    {
        route_event_queue.push(event_type);
    }
    
    lanelet::Optional<lanelet::routing::Route> RouteGeneratorWorker::reroute_after_route_invalidation(std::vector<lanelet::BasicPoint2d>& destination_points_in_map)
    {
        std::vector<lanelet::BasicPoint2d> destination_points_in_map_temp;
        
        for(const auto &i:destination_points_in_map) // Identify all route points that we have not yet passed
        {
            double destination_down_track=world_model_->routeTrackPos(i).downtrack;
            
            if( current_downtrack_distance_< destination_down_track)
            {
                destination_points_in_map_temp.push_back(i);
                ROS_DEBUG_STREAM("current_downtrack_distance_:" << current_downtrack_distance_);
                ROS_DEBUG_STREAM("destination_down_track:" << destination_down_track);
            }
        }  
        
        destination_points_in_map_ = destination_points_in_map_temp; // Update our route point list
        
        ROS_DEBUG_STREAM("New destination_points_in_map.size:" << destination_points_in_map_.size());

        auto route=routing(current_loc_, // Route from current location through future destinations
                            std::vector<lanelet::BasicPoint2d>(destination_points_in_map_.begin(), destination_points_in_map_.end() - 1),
                            destination_points_in_map_.back(),
                            world_model_->getMap(), world_model_->getMapRoutingGraph());

        return route;
    }

    bool RouteGeneratorWorker::spin_callback()
    {
        if(reroutingChecker()==true)
        {
           this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_INVALIDATION);
           publish_route_event(cav_msgs::RouteEvent::ROUTE_INVALIDATION);
           auto route = reroute_after_route_invalidation(destination_points_in_map_);

           // check if route successed
           if(!route)
            {
                ROS_ERROR_STREAM("Cannot find a route passing all destinations.");
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }
            else if(check_for_duplicate_lanelets_in_shortest_path(route.get()))
            {
                ROS_ERROR_STREAM("At least one duplicate Lanelet ID occurs in the shortest path. Routing cannot be completed.");
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return true;
            }
            else
            {
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_STARTED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_STARTED);  
            }    
            std::string original_route_name = route_msg_.route_name;
            route_msg_=compose_route_msg(route);
            route_msg_.route_name = original_route_name;
            route_msg_.is_rerouted = true;
            route_msg_.map_version = world_model_->getMapVersion();
            route_marker_msg_=compose_route_marker_msg(route);
            new_route_msg_generated_=true;
            new_route_marker_generated_=true;
        }
        
        // publish new route and set new route flag back to false
        if(new_route_msg_generated_ && new_route_marker_generated_)
        {
            route_pub_.publish(route_msg_);
            route_marker_pub_.publish(route_marker_msg_);
            new_route_msg_generated_ = false;
            new_route_marker_generated_ = false;
            route_msg_.is_rerouted = false;
        }
        // publish route state messsage if a route is selected
        if(route_msg_.route_name != "")
        {
            cav_msgs::RouteState state_msg;
            state_msg.header.stamp = ros::Time::now();
            state_msg.routeID = route_msg_.route_name;
            state_msg.cross_track = current_crosstrack_distance_;
            state_msg.down_track = current_downtrack_distance_;
            state_msg.lanelet_downtrack = ll_downtrack_distance_;            
            state_msg.state = this->rs_worker_.get_route_state();
            state_msg.lanelet_id = ll_id_;
            state_msg.speed_limit = speed_limit_;
            route_state_pub_.publish(state_msg);
        }
        // publish route event in order if any
        while(!route_event_queue.empty())
        {
            route_event_msg_.event = route_event_queue.front();
            route_event_pub_.publish(route_event_msg_);
            route_event_queue.pop();
        }
        return true; 
    }

    bool RouteGeneratorWorker::crosstrack_error_check(const geometry_msgs::PoseStampedConstPtr& msg, lanelet::ConstLanelet current)
    {
       lanelet::BasicPoint2d position;

        position.x()= msg->pose.position.x;
        position.y()= msg->pose.position.y;

        if(boost::geometry::within(position, current.polygon2d())) //If vehicle is inside current_lanelet, there is no crosstrack error
        {
            cte_count_ = 0;
            return false;
        }

        ROS_DEBUG_STREAM("LLt Polygon Dimensions1: " << current.polygon2d().front().x()<< ", "<< current.polygon2d().front().y());
        ROS_DEBUG_STREAM("LLt Polygon Dimensions2: " << current.polygon2d().back().x()<< ", "<< current.polygon2d().back().y());
        ROS_DEBUG_STREAM("Distance1: "<< boost::geometry::distance(position, current.polygon2d())<<" Crosstrack: "<< cross_track_dist );
    
        if (boost::geometry::distance(position, current.polygon2d()) > cross_track_dist) //Evaluate lanelet crosstrack distance from vehicle
            {
                cte_count_++;

                if(cte_count_ > cte_count_max_) //If the distance exceeds the crosstrack distance a certain number of times, report that the route has been departed
                    {
                        cte_count_ = 0;
                        return true;
                    }
                else 
                    return false;
            }
        else
            {
                cte_count_ = 0;
                return false;
            }
 
    }



    lanelet::ConstLanelet RouteGeneratorWorker::get_closest_lanelet_from_route_llts(lanelet::BasicPoint2d position)
    {
       double min = std::numeric_limits<double>::infinity();
        lanelet::ConstLanelet min_llt;
        for (auto i: route_llts)
         {
            double dist = boost::geometry::distance(position, i.polygon2d());
            if (dist < min)
            {
                min = dist;
                min_llt = i;
            }
     }
    return min_llt;
    }

    void RouteGeneratorWorker::set_CTE_dist(double cte_dist)
    {
        cross_track_dist = cte_dist;
    }

     void RouteGeneratorWorker::set_CTE_count_max(int cte_max)
    {
        cte_count_max_ = cte_max;

    }

    void RouteGeneratorWorker::addllt(lanelet::ConstLanelet llt)
    {
        route_llts.push_back(llt);
    }


}

