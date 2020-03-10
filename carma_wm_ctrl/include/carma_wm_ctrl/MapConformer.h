#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
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

#include <carma_wm/WorldModel.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_extension/regulatory_elements/RegionAccessRule.h>
#include <lanelet2_extension/regulatory_elements/PassingControlLine.h>
#include <lanelet2_extension/regulatory_elements/DirectionOfTravel.h>

namespace lanelet
{
/**
 * The map conformer API is responsible for ensuring that a loaded lanelet2 map meets the expectations of
 * CarmaUsTrafficRules Rather than simply validating the map this class will actually modify the map when possible to
 * ensure compliance.
 */
namespace MapConformer
{
/**
 * @brief Function modifies an existing map to make a best effort attempt at ensuring the map confroms to the
 * expectations of CarmaUSTrafficRules
 *
 * Map is updated by ensuring all lanelet and area bounds are marked with PassingControlLines
 * In addition, lanelets and areas are updated to have their accessability marked with a RegionAccessRule.
 * At the moment the creation of DigitalSpeedLimits for all lanelets/areas is not performed. This is because
 * CarmaUSTrafficRules supports the existing SpeedLimit definition and allows DigitalSpeedLimits to be overlayed on
 * that.
 *
 * @param map A pointer to the map which will be modified in place
 */
void ensureCompliance(lanelet::LaneletMapPtr map);
};  // namespace MapConformer
}  // namespace lanelet