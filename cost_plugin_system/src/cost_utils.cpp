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

#include "cost_utils.hpp"
#include <cav_msgs/Maneuver.h>
#include <exception>

namespace cost_utils
{
ros::Time get_plan_end_time(const cav_msgs::ManeuverPlan &plan)
{
    if (plan.maneuvers.empty())
    {
        throw std::invalid_argument("cost_plugin_system::get_plan_end_time called on empty maneuver plan");
    }

    cav_msgs::Maneuver m = plan.maneuvers.back();

    return get_maneuver_end_time(m);
}

double get_plan_end_distance(const cav_msgs::ManeuverPlan &plan)
{
    if (plan.maneuvers.empty())
    {
        throw std::invalid_argument("cost_plugin_system::get_plan_end_dist called on empty maneuver plan");
    }

    cav_msgs::Maneuver m = plan.maneuvers.back();
    return get_maneuver_end_distance(m);
}

ros::Time get_plan_start_time(const cav_msgs::ManeuverPlan &plan)
{
    if (plan.maneuvers.empty())
    {
        throw std::invalid_argument("cost_plugin_system::get_plan_start_time called on empty maneuver plan");
    }

    cav_msgs::Maneuver m = plan.maneuvers.front();
    return get_maneuver_start_time(m);
}

double get_plan_start_distance(const cav_msgs::ManeuverPlan &plan)
{
    if (plan.maneuvers.empty())
    {
        throw std::invalid_argument("cost_plugin_system::get_plan_start_dist called on empty maneuver plan");
    }

    cav_msgs::Maneuver m = plan.maneuvers.front();
    return get_maneuver_start_distance(m);
}

ros::Time get_maneuver_end_time(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, end_time);
}

ros::Time get_maneuver_start_time(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, start_time);
}

double get_maneuver_end_distance(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, end_dist);
}

double get_maneuver_start_distance(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, start_dist);
}

double get_maneuver_start_speed(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, start_speed);
}

double get_maneuver_end_speed(const cav_msgs::Maneuver &mvr)
{
    return GET_MANEUVER_PROPERTY(mvr, end_speed);
}

std::string get_maneuver_starting_lane_id(const cav_msgs::Maneuver &mvr)
{
    if (mvr.type == cav_msgs::Maneuver::LANE_FOLLOWING){
        return mvr.lane_following_maneuver.lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN) {
        return mvr.intersection_transit_left_turn_maneuver.starting_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN) {
        return mvr.intersection_transit_right_turn_maneuver.starting_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT) {
        return mvr.intersection_transit_straight_maneuver.starting_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::LANE_CHANGE) {
        return mvr.lane_change_maneuver.starting_lane_id;
    } 

    throw std::invalid_argument("Trying to get starting_lane_id of maneuver with invalid type.");
}

std::string get_maneuver_ending_lane_id(const cav_msgs::Maneuver &mvr)
{
    if (mvr.type == cav_msgs::Maneuver::LANE_FOLLOWING){
        return mvr.lane_following_maneuver.lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN) {
        return mvr.intersection_transit_left_turn_maneuver.ending_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN) {
        return mvr.intersection_transit_right_turn_maneuver.ending_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT) {
        return mvr.intersection_transit_straight_maneuver.ending_lane_id;
    } else if (mvr.type == cav_msgs::Maneuver::LANE_CHANGE) {
        return mvr.lane_change_maneuver.ending_lane_id;
    } 

    throw std::invalid_argument("Trying to get ending_lane_id of maneuver with invalid type.");
}

} // namespace cost_utils
