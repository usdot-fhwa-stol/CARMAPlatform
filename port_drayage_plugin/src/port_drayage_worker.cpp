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

#include "port_drayage_plugin/port_drayage_worker.h"
#include "ros/ros.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace port_drayage_plugin
{
    bool PortDrayageWorker::check_for_stop(const cav_msgs::ManeuverPlanConstPtr& plan, const geometry_msgs::TwistStampedConstPtr& speed) const {
        if (plan == nullptr || speed == nullptr) {
            ROS_WARN("Checking for stop when PortDrayagePlugin not properly initialized. Speed or plan is null");
            return false;
        }

        if (plan->maneuvers.size() > 0 && plan->maneuvers[0].type == cav_msgs::Maneuver::STOP_AND_WAIT) {
            cav_msgs::Maneuver stop_maneuver = _cur_plan->maneuvers[0];
            if (stop_maneuver.stop_and_wait_maneuver.parameters.planning_strategic_plugin == PORT_DRAYAGE_PLUGIN_ID) {
                double longitudinal_speed = speed->twist.linear.x; 
                if (longitudinal_speed <= _stop_speed_epsilon) {
                    return true;
                }
            }
        }

        return false;
    }

    // @SONAR_STOP@
    bool PortDrayageWorker::spin() {
        if (check_for_stop(_cur_plan, _cur_speed)) {
            _pdsm.process_event(PortDrayageEvent::ARRIVED_AT_DESTINATION);
        }

        return true;
    }

    void PortDrayageWorker::set_maneuver_plan(const cav_msgs::ManeuverPlanConstPtr& plan) {
        _cur_plan = plan;
    }

    void PortDrayageWorker::set_current_speed(const geometry_msgs::TwistStampedConstPtr& speed) {
        _cur_speed = speed;
    }

    void PortDrayageWorker::initialize() {
        _pdsm.set_on_arrived_at_destination_callback(std::bind(&PortDrayageWorker::on_arrived_at_destination, this));
        _pdsm.set_on_received_new_destination_callback(std::bind(&PortDrayageWorker::on_received_new_destination, this));
    }
    // @SONAR_START@

    void PortDrayageWorker::on_arrived_at_destination() {
        cav_msgs::MobilityOperation msg = compose_arrival_message();
        _publish_mobility_operation(msg);
    }

    void PortDrayageWorker::on_received_new_destination() {       
        //  Populate the service request with the destination coordinates from the last received port drayage mobility operation message
        cav_srvs::SetActiveRoute route_req = compose_set_active_route_request();

        // Call service client to set the new active route
        bool is_route_generation_successful = _call_set_active_route_client(route_req);

        // Throw exception if route generation was not successful
        if (!is_route_generation_successful) {
            throw std::invalid_argument("Route generation failed. Routing could not be completed.");
        }
    }

    cav_srvs::SetActiveRoute PortDrayageWorker::compose_set_active_route_request() const {
        cav_srvs::SetActiveRoute route_req;
        if (_latest_mobility_operation_msg.dest_latitude && _latest_mobility_operation_msg.dest_longitude) {
            route_req.request.choice = cav_srvs::SetActiveRouteRequest::DESTINATION_POINTS_ARRAY;

            cav_msgs::Position3D destination_point;
            destination_point.latitude = *_latest_mobility_operation_msg.dest_latitude;
            destination_point.longitude = *_latest_mobility_operation_msg.dest_longitude;
            destination_point.elevation_exists = false;
            
            route_req.request.destination_points.push_back(destination_point);
        }
        else {
            throw std::invalid_argument("No destination points were received. Routing could not be completed");
        }

        return route_req;
    }

    cav_msgs::MobilityOperation PortDrayageWorker::compose_arrival_message() const {
        cav_msgs::MobilityOperation msg;

        msg.header.plan_id = "";
        msg.header.sender_id = _host_id;
        msg.header.sender_bsm_id = _host_bsm_id;
        msg.header.recipient_id = "";
        msg.header.timestamp = ros::Time::now().toNSec();

        msg.strategy = PORT_DRAYAGE_STRATEGY_ID;

        // Encode JSON with Boost Property Tree
        using boost::property_tree::ptree;
        ptree pt;
        pt.put("cmv_id", _cmv_id);
        pt.put("cargo_id", _cargo_id);
        pt.put("operation", PORT_DRAYAGE_ARRIVAL_OPERATION_ID);
        std::stringstream body_stream;
        boost::property_tree::json_parser::write_json(body_stream, pt);
        msg.strategy_params = body_stream.str();

        return msg;
    }

    void PortDrayageWorker::on_inbound_mobility_operation(const cav_msgs::MobilityOperationConstPtr& msg) {
        // Check if the received message is a new message for port drayage
        if((msg->strategy == PORT_DRAYAGE_STRATEGY_ID) && (msg->strategy_params != _previous_strategy_params)) {
            // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
            using boost::property_tree::ptree;
            ptree pt;
            std::istringstream strategy_params_ss(msg->strategy_params);
            boost::property_tree::json_parser::read_json(strategy_params_ss, pt);
            std::string mobility_operation_cmv_id = pt.get<std::string>("cmv_id");

            // Check if the received MobilityOperation message is intended for this vehicle's cmv_id   
            if(mobility_operation_cmv_id == _cmv_id) {
                ROS_DEBUG_STREAM("Processing new port drayage MobilityOperation message for cmv_id " << mobility_operation_cmv_id);
                mobility_operation_message_parser(msg->strategy_params);  
                _previous_strategy_params = msg->strategy_params;
                
                // Process event based on the PortDrayageEvent associated with the received MobilityOperation message
                switch(_latest_mobility_operation_msg.port_drayage_event_type) {
                    case PortDrayageEvent::RECEIVED_NEW_DESTINATION:
                        ROS_DEBUG_STREAM("Processing RECEIVED_NEW_DESTINATION event for operation type " << _latest_mobility_operation_msg.operation);
                        _pdsm.process_event(PortDrayageEvent::RECEIVED_NEW_DESTINATION);
                        break;
                    default:
                        ROS_DEBUG_STREAM("Not processing an event for operation type " << _latest_mobility_operation_msg.operation);
                        break;
                }
            }
            else {
                ROS_DEBUG_STREAM("Ignoring received port drayage MobilityOperation message intended for cmv_id " << mobility_operation_cmv_id);
            }
        }
    }

    void PortDrayageWorker::mobility_operation_message_parser(std::string mobility_operation_strategy_params) {
        // Use Boost Property Tree to parse JSON-encoded strategy_params field in MobilityOperations message
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream mobility_operation_strategy_params_ss(mobility_operation_strategy_params);
        boost::property_tree::json_parser::read_json(mobility_operation_strategy_params_ss, pt);

        _latest_mobility_operation_msg.cargo_id = pt.get<std::string>("cargo_id");
        _latest_mobility_operation_msg.has_cargo = pt.get<bool>("cargo");
        _latest_mobility_operation_msg.current_action_id = pt.get<std::string>("action_id");
        _latest_mobility_operation_msg.next_action_id = pt.get<std::string>("next_action");
        _latest_mobility_operation_msg.operation = pt.get<std::string>("operation");

        if(_latest_mobility_operation_msg.operation == "MOVING_TO_LOADING_AREA") {
            _latest_mobility_operation_msg.port_drayage_event_type = PortDrayageEvent::RECEIVED_NEW_DESTINATION;
        }

        // Parse starting longitude/latitude fields if 'location' field exists in strategy_params:
        if (pt.count("location") != 0){
            _latest_mobility_operation_msg.start_longitude = pt.get<double>("location.longitude") / 10000000; // Convert 1/10 microdegrees to degrees
            _latest_mobility_operation_msg.start_latitude = pt.get<double>("location.latitude") / 10000000; // Convert 1/10 microdegrees to degrees
            ROS_DEBUG_STREAM("start long: " << *_latest_mobility_operation_msg.start_longitude);
            ROS_DEBUG_STREAM("start lat: " << *_latest_mobility_operation_msg.start_latitude);
        }
        else {
            _latest_mobility_operation_msg.start_longitude = boost::optional<double>();
            _latest_mobility_operation_msg.start_latitude = boost::optional<double>();
        }

        // Parse destination longitude/latitude fields if 'destination' field exists in strategy_params:
        if(pt.count("destination") != 0) {
            _latest_mobility_operation_msg.dest_longitude = pt.get<double>("destination.longitude") / 10000000; // Convert 1/10 microdegrees to degrees
            _latest_mobility_operation_msg.dest_latitude = pt.get<double>("destination.latitude") / 10000000; // Convert 1/10 microdegrees to degrees
            ROS_DEBUG_STREAM("dest long: " << *_latest_mobility_operation_msg.dest_longitude);
            ROS_DEBUG_STREAM("dest lat: " << *_latest_mobility_operation_msg.dest_latitude);
        }
        else {
            _latest_mobility_operation_msg.dest_longitude = boost::optional<double>();
            _latest_mobility_operation_msg.dest_latitude = boost::optional<double>();
        }

        ROS_DEBUG_STREAM("cargo id: " << _latest_mobility_operation_msg.cargo_id);
        ROS_DEBUG_STREAM("operation: " << _latest_mobility_operation_msg.operation);
        ROS_DEBUG_STREAM("cargo flag: " << _latest_mobility_operation_msg.has_cargo);
        ROS_DEBUG_STREAM("current action id: " << _latest_mobility_operation_msg.current_action_id);
        ROS_DEBUG_STREAM("next action id: " << _latest_mobility_operation_msg.next_action_id);
    }

} // namespace port_drayage_plugin
