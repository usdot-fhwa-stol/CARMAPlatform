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

#include "port_drayage_plugin/port_drayage_plugin.h"
#include "port_drayage_plugin/port_drayage_worker.h"

namespace port_drayage_plugin
{

    // @SONAR_STOP@
    int PortDrayagePlugin::run() {
        if (_nh == nullptr || _pnh == nullptr) {
            ROS_ERROR("Port Drayage Plugin not properly initialized, node handles are null!");
            return -1;
        }

        double speed_epsilon = _pnh->param("stop_speed_epsilon", 1.0);
        declaration = _pnh->param("declaration", 1.0);
        std::string cmv_id;
        _pnh->param<std::string>("cmv_id", cmv_id, "");
        std::string cargo_id;
        _pnh->param<std::string>("cargo_id", cargo_id, "");

        ros::Publisher outbound_mob_op = _nh->advertise<cav_msgs::MobilityOperation>("outbound_mobility_operation", 5);
        _outbound_mobility_operations_publisher = std::make_shared<ros::Publisher>(outbound_mob_op);
        PortDrayageWorker pdw{
            cmv_id,
            cargo_id,
            "HOST_ID",
            [this](cav_msgs::MobilityOperation msg) {
               _outbound_mobility_operations_publisher->publish<cav_msgs::MobilityOperation>(msg);
            },
            speed_epsilon
        };
        
        ros::Subscriber maneuver_sub = _nh->subscribe<cav_msgs::ManeuverPlan>("final_Maneuver_plan", 5, 
            [&](const cav_msgs::ManeuverPlanConstPtr& plan) {
                pdw.set_maneuver_plan(plan);
        });
        _maneuver_plan_subscriber = std::make_shared<ros::Subscriber>(maneuver_sub);

        ros::Subscriber twist_sub = _nh->subscribe<geometry_msgs::TwistStamped>("localization/ekf_twist", 5, 
            [&](const geometry_msgs::TwistStampedConstPtr& speed) {
                pdw.set_current_speed(speed);
                _cur_speed = speed->twist;
        });
        _cur_speed_subscriber = std::make_shared<ros::Subscriber>(twist_sub);

        plan_maneuver_srv_ = _nh->advertiseService("strategic_plan/plan_maneuvers", &PortDrayagePlugin::plan_maneuver_cb, this);

        ros::Subscriber pose_sub = _nh->subscribe<geometry_msgs::PoseStamped>("current_pose", 5, 
            [&](const geometry_msgs::PoseStampedConstPtr& pose) {
                curr_pose_ = std::make_shared<geometry_msgs::PoseStamped>(*pose);
        });

        _pose_subscriber = std::make_shared<ros::Subscriber>(pose_sub);

        std::function<bool()> spin_cb = [&]() {
            return pdw.spin();
        };
        _nh->setSpinCallback(spin_cb);

        ros::CARMANodeHandle::spin();

        return 0;
    }

    bool PortDrayagePlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp){

        if(curr_pose_ == nullptr) {
            return false;
        }

        lanelet::BasicPoint2d current_loc(curr_pose_->pose.position.x, curr_pose_->pose.position.y);

        if(wm_ == nullptr) {
            return false;
        }

        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 1);

        if(current_lanelets.size() == 0) {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return false;
        }

        auto current_lanelet = current_lanelets[0];
        auto stop_rules = current_lanelet.second.regulatoryElementsAs<lanelet::StopRule>();
        double stop_rule_downtrack = 0;

        if(stop_rules.empty()) {
            return false;
        }
        else {
            auto stop_rule_elem = stop_rules.front();
            auto stop_line_vector = stop_rule_elem->stopAndWaitLine();
            auto stop_line = lanelet::traits::to2D(stop_line_vector.front());
            auto point = lanelet::traits::to2D(stop_line.front());
            auto pos = carma_wm::geometry::trackPos(current_lanelet.second, point);
            stop_rule_downtrack = pos.downtrack;
        }

        double current_loc_downtrack = wm_->routeTrackPos(current_loc).downtrack;

        double speed_progress = _cur_speed.linear.x;
        double current_progress = 0;
        
        if(current_loc_downtrack < stop_rule_downtrack)
        {

            ros::Time start_time;

            if(req.prior_plan.maneuvers.size() > 0) {
                switch(req.prior_plan.maneuvers.back().type) {
                    case cav_msgs::Maneuver::LANE_FOLLOWING : 
                        start_time = req.prior_plan.maneuvers.back().lane_following_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().lane_following_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::LANE_CHANGE : 
                        start_time = req.prior_plan.maneuvers.back().lane_change_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().lane_change_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_straight_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_straight_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_left_turn_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_left_turn_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_right_turn_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_right_turn_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::STOP_AND_WAIT : 
                        start_time = req.prior_plan.maneuvers.back().stop_and_wait_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().stop_and_wait_maneuver.end_dist;
                        break;
                }
            } else {
                start_time = ros::Time::now();
                current_progress = current_loc_downtrack;
            }

            double end_dist = stop_rule_downtrack;

            double estimated_distance_to_stop = estimate_distance_to_stop(speed_progress,declaration);
            double estimated_time_to_stop = estimate_time_to_stop(estimated_distance_to_stop,speed_progress);

            double lane_following_distance = stop_rule_downtrack - estimated_distance_to_stop;
            double stop_and_wait_distance = estimated_distance_to_stop;

            resp.new_plan.maneuvers.push_back(
                compose_lane_following_maneuver_message(current_progress, 
                                       current_progress + lane_following_distance, 
                                       speed_progress, 
                                       speed_progress, 
                                       current_lanelet.second.id(), 
                                       start_time));

            resp.new_plan.maneuvers.push_back(
                compose_stop_and_wait_maneuver_message(current_progress + lane_following_distance, 
                                       end_dist, 
                                       speed_progress, 
                                       0.0, 
                                       current_lanelet.second.id(), 
                                       start_time,
                                       estimated_time_to_stop));
        }

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }

        return true;
    };

    cav_msgs::Maneuver PortDrayagePlugin::compose_stop_and_wait_maneuver_message(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time time, double time_to_stop)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "StopAndWaitPlugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "PortDrayageWorkerPlugin";
        maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = current_speed;
        maneuver_msg.stop_and_wait_maneuver.start_time = time;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.end_time = time + ros::Duration(std::max(15.0,time_to_stop));
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(lane_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(lane_id);
        return maneuver_msg;
    }

    cav_msgs::Maneuver PortDrayagePlugin::compose_lane_following_maneuver_message(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.neogition_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InlaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "RouteFollowingPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = time + ros::Duration((end_dist - current_dist) / (0.5 * (current_speed + target_speed)));
        maneuver_msg.lane_following_maneuver.lane_id = std::to_string(lane_id);
        return maneuver_msg;
    }
    // @SONAR_START@

    double estimate_distance_to_stop(double v, double a) {
        return (v*v)/2*a;
    }

    double estimate_time_to_stop(double d, double v) {
        return 2*d/v;
    };

} // namespace port_drayage_plugin