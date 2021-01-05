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

#include "tree_planner.hpp"
#include "arbitrator_utils.hpp"
#include <vector>
#include <map>
#include <limits>

namespace arbitrator
{
    cav_msgs::ManeuverPlan TreePlanner::generate_plan() 
    {
        cav_msgs::ManeuverPlan root;
        std::vector<std::pair<cav_msgs::ManeuverPlan, double>> open_list;
        const double INF = std::numeric_limits<double>::infinity();
        open_list.push_back(std::make_pair(root, INF));

        cav_msgs::ManeuverPlan longest_plan = root; // Track longest plan in case target length is never reached
        ros::Duration longest_plan_duration = ros::Duration(0);

        while (!open_list.empty())
        {
            std::vector<std::pair<cav_msgs::ManeuverPlan, double>> new_open_list;
            for (auto it = open_list.begin(); it != open_list.end(); it++)
            {
                // Pop the first element off the open list
                cav_msgs::ManeuverPlan cur_plan = it->first;
                ros::Duration plan_duration; // zero duration

                // If we're not at the root, plan_duration is nonzero (our plan should have maneuvers)
                if (!cur_plan.maneuvers.empty()) 
                {
                    // get plan duration
                    plan_duration = arbitrator_utils::get_plan_end_time(cur_plan) - arbitrator_utils::get_plan_start_time(cur_plan); 
                }
                // Evaluate terminal condition
                if (plan_duration >= target_plan_duration_) 
                {
                    return cur_plan;
                } else if (plan_duration > longest_plan_duration) {
                    longest_plan_duration = plan_duration;
                    longest_plan = cur_plan;
                }

                // Expand it, and reprioritize
                std::vector<cav_msgs::ManeuverPlan> children = neighbor_generator_.generate_neighbors(cur_plan);
                
                // Compute cost for each child and store in open list
                for (auto child = children.begin(); child != children.end(); child++)
                {
                    if (child->maneuvers.empty())
                        continue;   
                    new_open_list.push_back(std::make_pair(*child, cost_function_.compute_cost_per_unit_distance(*child)));
                }
            }
            
            new_open_list = search_strategy_.prioritize_plans(new_open_list);
            open_list = new_open_list;
        }


        // If no perfect match is found, return the longest plan that fit the criteria
        return longest_plan;
    }
}
