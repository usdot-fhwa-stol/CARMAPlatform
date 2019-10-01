/*
 * Copyright (C) 2019 LEIDOS.
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

#include <ros/ros.h>
#include <memory>
#include "arbitrator.hpp"
#include "arbitrator_state_machine.hpp"
#include "fixed_priority_cost_function.hpp"
#include "plugin_neighbor_generator.hpp"
#include "beam_search_strategy.hpp"
#include "tree_planner.hpp"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "arbitrator");
    ros::CARMANodeHandle nh = ros::CARMANodeHandle("arbitrator");
    ros::CARMANodeHandle pnh = ros::CARMANodeHandle("~");

    // Handle dependency injection
    arbitrator::CapabilitiesInterface ci{nh};
    arbitrator::ArbitratorStateMachine sm;
    arbitrator::FixedPriorityCostFunction fpcf{std::map<std::string, double>()};
    arbitrator::BeamSearchStrategy bss{3};
    arbitrator::PluginNeighborGenerator png{ci};
    arbitrator::TreePlanner tp{fpcf, png, bss};
    arbitrator::Arbitrator arbitrator{nh, pnh, sm, ci, tp};

    arbitrator.run();

    return 0;
}
