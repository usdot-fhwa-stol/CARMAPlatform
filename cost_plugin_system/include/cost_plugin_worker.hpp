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

#pragma once

#include <string>
#include <ros/ros.h>
#include <atomic>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/ComputePlanCost.h>
#include "cost_safety.hpp"
#include "cost_fuel.hpp"
#include "cost_legality.hpp"
#include "cost_comfort.hpp"
#include "cost_efficiency.hpp"
#include "cost_feasibility.hpp"

namespace cost_plugin_system
{
class CostPluginWorker
{
public:
    /*!
     * \brief Default constructor for CostPluginWorker
     */
    CostPluginWorker();

    /**
     * \brief Initialize the cost plugin system
     */
    void init();

    /**
     * \brief Run the spin loop of cost plugin system
     */
    void run();

    // Node handles
    std::unique_ptr<ros::CARMANodeHandle> nh_;
    std::unique_ptr<ros::CARMANodeHandle> pnh_;

    // Service servers
    ros::ServiceServer compute_plan_cost_service_server_;

    double compute_final_score(cav_msgs::ManeuverPlan plan);
private:
    double max_accelaration_;
    double max_decelaration_;
    double speed_limit_;
    double speed_buffer_;
    double weight_of_comfort_;
    double weight_of_efficiency_;
    double weight_of_feasibility_;
    double weight_of_fuel_;
    double weight_of_safety_;

    bool get_score(cav_srvs::ComputePlanCostRequest& req, cav_srvs::ComputePlanCostResponse& res);
};
} // namespace cost_plugin_system