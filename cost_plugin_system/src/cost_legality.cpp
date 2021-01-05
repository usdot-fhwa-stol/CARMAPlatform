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

#include "cost_utils.hpp"
#include "cost_legality.hpp"

namespace cost_plugin_system
{

// TODO: There is no environment/infrastructure data to
//       this cost_plugin_system node now, so the compute_cost is empty.
//       This needs to be done later.
double CostofLegality::compute_cost(cav_msgs::ManeuverPlan plan) const
{

    double cost = 0.0;
    // TODO
    return cost;
}

} 