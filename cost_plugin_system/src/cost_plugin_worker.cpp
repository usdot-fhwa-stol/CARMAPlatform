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
#include "cost_plugin_worker.hpp"

namespace cost_plugin_system
{

    CostPluginWorker::CostPluginWorker(){
    }

void CostPluginWorker::init()
{
    nh_.reset(new ros::CARMANodeHandle());
    pnh_.reset(new ros::CARMANodeHandle("~"));

    pnh_->param<double>("max_accelaration", max_accelaration_, 5.0);
    pnh_->param<double>("max_decelaration", max_decelaration_, 8.0);

    pnh_->param<double>("speed_limit", speed_limit_, 27.0);
    pnh_->param<double>("speed_buffer", speed_buffer_, 25.0);

    pnh_->param<double>("weight_of_comfort", weight_of_comfort_, 1.0);
    pnh_->param<double>("weight_of_efficiency", weight_of_efficiency_, 1.0);
    pnh_->param<double>("weight_of_feasibility", weight_of_feasibility_, 1.0);
    pnh_->param<double>("weight_of_fuel", weight_of_fuel_, 1.0);
    pnh_->param<double>("weight_of_safety", weight_of_safety_, 1.0);
}

bool CostPluginWorker::get_score(cav_srvs::ComputePlanCostRequest& req, cav_srvs::ComputePlanCostResponse& res)
{
    cav_msgs::ManeuverPlan plan = req.maneuver_plan;

    res.plan_cost = compute_final_score(plan);

    return true;
}

double CostPluginWorker::compute_final_score(cav_msgs::ManeuverPlan plan)
{
    cost_plugin_system::CostofComfort coc(max_decelaration_);
    cost_plugin_system::CostofEfficiency coe(speed_limit_, speed_buffer_);
    cost_plugin_system::CostofFeasibility cofe(max_accelaration_, max_decelaration_);
    cost_plugin_system::CostofFuel cof;
    cost_plugin_system::CostofLegality col;
    cost_plugin_system::CostofSafety cos(speed_limit_);

    double total_cost = 0.0;
    if (col.compute_cost(plan) == 0)
    {
        double cost_of_comfort = coc.compute_cost(plan);
        double cost_of_efficiency = coe.compute_cost(plan);
        double cost_of_feasibility = cofe.compute_cost(plan);
        double cost_of_fuel = cof.compute_cost(plan);
        double cost_of_safety = cos.compute_cost(plan);

        total_cost = weight_of_comfort_ * cost_of_comfort + weight_of_efficiency_ * cost_of_efficiency +
                     weight_of_feasibility_ * cost_of_feasibility + weight_of_fuel_ * cost_of_fuel +
                     weight_of_safety_ * cost_of_safety;
    }
    else
    {
        total_cost = -999.0;
    }
    return total_cost;
}

void CostPluginWorker::run()
{
    init();

    ROS_INFO("Initalizing cost_plugin_system node...");
    // Init our ROS objects
    compute_plan_cost_service_server_ = nh_->advertiseService("compute_plan_cost", &CostPluginWorker::get_score, this);
    ROS_INFO("Ready to compute the total cost");
    ros::spin();
}
} // namespace cost_plugin_system