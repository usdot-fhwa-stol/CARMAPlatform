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
#include <limits>
#include <math.h>
#include "route_generator_worker.h"

namespace route {

    RouteGeneratorWorker::RouteGeneratorWorker(tf2_ros::Buffer& tf_buffer) :
                                               tf_tree_(tf_buffer) { }
    
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
                    } else
                    {
                        ROS_ERROR_STREAM("File open failed...");
                    }
                    auto last_comma = dest_name.find_last_of(',');
                    route_msg.route_name = dest_name.substr(last_comma + 1);
                    resp.availableRoutes.push_back(route_msg);
                }
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
        // only allow activate a new route in route selection state
        if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::SELECTION)
        {
            // entering to routing state once destinations are picked
            this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_SELECTED);
            publish_route_event(cav_msgs::RouteEvent::ROUTE_SELECTED);
            // get transform from ECEF(earth) to local map frame
            tf2::Transform map_in_earth;
            try
            {
                tf2::convert(tf_tree_.lookupTransform("earth", "map", ros::Time(0)).transform, map_in_earth);
            }
            catch (const tf2::TransformException &ex)
            {
                ROS_ERROR_STREAM("Could not lookup transform with exception " << ex.what());
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::TRANSFORM_ERROR;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return false;
            }

            // load destination points in ECEF frame
            auto destination_points = load_route_destinations_in_ecef(req.routeID);
            // Check if route file are valid with at least one starting points and one destination points
            if(destination_points.size() < 2)
            {
                ROS_ERROR_STREAM("Selected route file contains 1 or less points. Routing cannot be completed.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTE_FILE_ERROR;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return false;
            }
            // convert points in ECEF to map frame
            auto destination_points_in_map = transform_to_map_frame(destination_points, map_in_earth);
            int idx = 0;
            // validate if the points are geometrically in the map
            for (auto pt : destination_points_in_map)
            {
                auto llts = world_model_->getLaneletsFromPoint(pt, 1);
                if (llts.empty())
                {
                    ROS_ERROR_STREAM("Route Generator: " << idx 
                        << "th destination point is not in the map, x: " << pt.x() << " y: " << pt.y());
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTE_FILE_ERROR;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return false;
                }
                idx ++;
            }
            
            // get route graph from world model object
            auto p = world_model_->getMapRoutingGraph();
            // generate a route
            auto route = routing(destination_points_in_map.front(),
                                std::vector<lanelet::BasicPoint2d>(destination_points_in_map.begin() + 1, destination_points_in_map.end() - 1),
                                destination_points_in_map.back(),
                                world_model_->getMap(), world_model_->getMapRoutingGraph());
            // check if route successed
            if(!route)
            {
                ROS_ERROR_STREAM("Cannot find a route passing all destinations.");
                resp.errorStatus = cav_srvs::SetActiveRouteResponse::ROUTING_FAILURE;
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_GEN_FAILED);
                return false;
            }
            // update route message
            route_msg_ = compose_route_msg(route);
            route_marker_msg_ = compose_route_marker_msg(route);
            route_msg_.header.stamp = ros::Time::now();
            route_msg_.header.frame_id = "map";
            route_msg_.route_name = req.routeID;
            // since routing is done correctly, transit to route following state
            this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_STARTED);
            publish_route_event(cav_msgs::RouteEvent::ROUTE_STARTED);
            // set publish flag such that updated msg will be published in the next spin
            new_route_msg_generated_ = true;
            return true;
        }
        return false;
    }

    std::vector<tf2::Vector3> RouteGeneratorWorker::load_route_destinations_in_ecef(const std::string& route_id) const
    {
        // compose full path of the route file
        std::string route_file_name = route_file_path_ + route_id + ".csv";
        std::ifstream fs(route_file_name);
        std::string line;
        std::vector<tf2::Vector3> destination_points;
        // read each line if any
        while(std::getline(fs, line))
        {
            wgs84_utils::wgs84_coordinate coordinate;
            // lat lon and elev is seperated by comma
            auto comma = line.find(",");
            // convert lon value in the range of [0, 360.0] degree and then into rad
            coordinate.lon =
                (std::stod(line.substr(0, comma)) < 0 ? std::stod(line.substr(0, comma)) + 360.0 : std::stod(line.substr(0, comma))) * DEG_TO_RAD;
            line.erase(0, comma + 1);
            comma = line.find(",");
            // convert lat value in the range of [0, 360.0] degree and then into rad
            coordinate.lat =
                (std::stod(line.substr(0, comma)) < 0 ? std::stod(line.substr(0, comma)) + 360.0 : std::stod(line.substr(0, comma))) * DEG_TO_RAD;
            // elevation is in meters
            line.erase(0, comma + 1);
            comma = line.find(",");
            coordinate.elevation = std::stod(line.substr(0, comma));
            // no rotation needed since it only represents a point
            tf2::Quaternion no_rotation(0, 0, 0, 1);
            destination_points.emplace_back(wgs84_utils::geodesic_to_ecef(coordinate, tf2::Transform(no_rotation)));
        }
        return destination_points;
    }

    std::vector<lanelet::BasicPoint2d> RouteGeneratorWorker::transform_to_map_frame(const std::vector<tf2::Vector3>& ecef_points, const tf2::Transform& map_in_earth) const
    {
        std::vector<lanelet::BasicPoint2d> map_points;
        for(tf2::Vector3 point : ecef_points)
        {
            tf2::Transform point_in_earth;
            tf2::Quaternion no_rotation(0, 0, 0, 1);

            point_in_earth.setOrigin(point);
            point_in_earth.setRotation(no_rotation);
            // convert to map frame by (T_e_m)^(-1) * T_e_p
            auto point_in_map = map_in_earth.inverse() * point_in_earth;
            // return as 2D points as the API requiremnet of lanelet2 lib
            map_points.push_back(lanelet::BasicPoint2d(point_in_map.getOrigin().getX(), point_in_map.getOrigin().getY()));
        }
        return map_points;
    }

    visualization_msgs::MarkerArray RouteGeneratorWorker::compose_route_marker_msg(const lanelet::Optional<lanelet::routing::Route>& route)
    {
        std::vector<lanelet::ConstPoint3d> points;
        for(const auto& ll : route.get().shortestPath())
        {
            for(const auto& pt : ll.centerline())
            {
                points.push_back(pt);
            }
        }

        route_marker_msg_.markers={};

        if (!points.empty())
        {
            ROS_WARN_STREAM("No central line points! Returning");
        }

        // create the marker msgs
        visualization_msgs::MarkerArray route_marker_msg;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.type = visualization_msgs::Marker::SPHERE;//
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "route_visualizer";

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.frame_locked = true;
 
        for (int i = 0; i < points.size(); i=i+5)
        {
            marker.id = i;

            marker.color.r = 1.0F;
            marker.color.g = 1.0F;
            marker.color.b = 1.0F;
            marker.color.a = 1.0F;

            marker.pose.position.x = points[i].x();
            marker.pose.position.y = points[i].y();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            
            route_marker_msg.markers.push_back(marker);
        }
        new_route_marker_generated_ = true;
        return route_marker_msg;
    }

    cav_msgs::Route RouteGeneratorWorker::compose_route_msg(const lanelet::Optional<lanelet::routing::Route>& route)
    {
        cav_msgs::Route msg;
        // iterate thought the shortest path to populat shortest_path_lanelet_ids
        for(const auto& ll : route.get().shortestPath())
        {
            msg.shortest_path_lanelet_ids.push_back(ll.id());
        }

        // iterate thought the all lanelet in the route to populat route_path_lanelet_ids
        for(const auto& ll : route.get().laneletSubmap()->laneletLayer)
        {
            msg.route_path_lanelet_ids.push_back(ll.id());
        }
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
        if(this->rs_worker_.get_route_state() == RouteStateWorker::RouteState::FOLLOWING) {
            // convert from pose stamp into lanelet basic 2D point
            lanelet::BasicPoint2d current_loc(msg->pose.position.x, msg->pose.position.y);
            // get dt ct from world model
            carma_wm::TrackPos track(0.0, 0.0);
            try {
                track = this->world_model_->routeTrackPos(current_loc);
            } catch (std::invalid_argument ex) {
                ROS_WARN_STREAM("Routing has finished but carma_wm has not receive it!");
                return;
            }
            auto via_lanelet_vector = lanelet::geometry::findNearest(world_model_->getMap()->laneletLayer, current_loc, 1);
            auto current_lanelet = lanelet::ConstLanelet(via_lanelet_vector[0].second.constData());
            auto lanelet_track = carma_wm::geometry::trackPos(current_lanelet, current_loc);
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
            if(std::fabs(current_crosstrack_distance_) > cross_track_max_)
            {
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_DEPARTED);
            }
            // check if we reached our destination be remaining down track distance
            if(current_downtrack_distance_ > world_model_->getRoute()->length2d() - down_track_target_range_)
            {
                this->rs_worker_.on_route_event(RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
                publish_route_event(cav_msgs::RouteEvent::ROUTE_COMPLETED);
            }
        }
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
    
    bool RouteGeneratorWorker::spin_callback()
    {
        // publish new route and set new route flag back to false
        if(new_route_msg_generated_ && new_route_marker_generated_)
        {
            route_pub_.publish(route_msg_);
            route_marker_pub_.publish(route_marker_msg_);
            new_route_msg_generated_ = false;
            new_route_marker_generated_ = false;
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

}

