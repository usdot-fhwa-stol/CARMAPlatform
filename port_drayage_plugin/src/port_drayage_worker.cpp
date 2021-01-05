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
    }
    // @SONAR_START@

    void PortDrayageWorker::on_arrived_at_destination() {
        cav_msgs::MobilityOperation msg = compose_arrival_message();
        _publish_mobility_operation(msg);
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


} // namespace port_drayage_plugin
