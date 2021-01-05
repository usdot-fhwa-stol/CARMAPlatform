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
#include <ros/ros.h>
#include <carma_wm/WorldModel.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/utility/Units.h>
#include <boost/algorithm/string.hpp>
#include <carma_wm/MapConformer.h>


namespace lanelet
{
/**
 * The map conformer is responsible for ensuring that a loaded lanelet2 map meets the expectations of
 * CarmaUsTrafficRules Rather than simply validating the map this class will actually modify the map when possible to
 * ensure compliance.
 * NOTE: This class is not meant to be used by carma_wm users at runtime. It is only used in WMTestLibraryForGuidance 
 * currently to support unit testing.
 */
namespace MapConformer
{
namespace
{  // Private namespace
// Enum defining types of lane change
enum class LaneChangeType
{
  ToRight,
  ToLeft,
  Both,
  None
};

// List of supported participants. Should exactly match elements of lanelet::Participants struct
constexpr size_t PARTICIPANT_COUNT = 12;
constexpr const char* participant_types[PARTICIPANT_COUNT] = { lanelet::Participants::Vehicle,  //
                                                               lanelet::Participants::VehicleBus,
                                                               lanelet::Participants::VehicleCar,
                                                               lanelet::Participants::VehicleCarElectric,
                                                               lanelet::Participants::VehicleCarCombustion,
                                                               lanelet::Participants::VehicleTruck,
                                                               lanelet::Participants::VehicleMotorcycle,
                                                               lanelet::Participants::VehicleTaxi,
                                                               lanelet::Participants::VehicleEmergency,
                                                               lanelet::Participants::Pedestrian,  //
                                                               lanelet::Participants::Bicycle,     //
                                                               lanelet::Participants::Train };

/**
 * @brief Helper function to return the set of all german traffic rules which are currently implemented
 *
 * @return A list of german traffic rules which have been implemented
 */
std::vector<lanelet::traffic_rules::TrafficRulesUPtr> getAllGermanTrafficRules()
{
  std::vector<lanelet::traffic_rules::TrafficRulesUPtr> german_traffic_rules_set;
  german_traffic_rules_set.reserve(PARTICIPANT_COUNT);

  for (size_t i = 0; i < PARTICIPANT_COUNT; i++)
  {
    // This loop is designed to identify the set of currently supported participants for german traffic rules
    // Using exceptions as logic like this is not usually a good practice, but since this is an operation performed only
    // at startup on a small number of elements efficiency should not be an issue
    try
    {
      lanelet::traffic_rules::TrafficRulesUPtr traffic_rules =
          lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, participant_types[i]);
      german_traffic_rules_set.emplace_back(std::move(traffic_rules));
    }
    catch (const lanelet::InvalidInputError& e)
    {
      // Ignore participants which there is no generic rules for
      ROS_INFO_STREAM ("Ignoring participant: " << participant_types[i] <<  ", which there is no generic rule for...");
    }
  }

  return german_traffic_rules_set;
}

/**
 * @brief Helper function to get a value from a map or return a default value when key is not present
 *
 * This function has been copied from lanelet2_traffic_rules/src/GenericTrafficRules.cpp as it was not exposed by the
 * Lanelet2 libraray. The function therefore has the same functionality as the GenericTrafficRules class
 *
 * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_traffic_rules/src/GenericTrafficRules.cpp
 * The source function is copyrighted under BSD 3-Clause "New" or "Revised" License a copy of that notice has been
 * included with this package
 *
 * @param map The map to search
 * @param key The key to evaluate
 * @param defaultVal The default to return if key is not in map
 *
 * @return Value at key in map or defaultVal if key is not in map
 */
template <typename Map, typename Key, typename Value>
Value getMapOrDefault(const Map& map, Key key, Value defaultVal)
{
  auto elem = map.find(key);
  if (elem == map.end())
  {
    return defaultVal;
  }
  return elem->second;
}

/**
 * @brief Helper function to check if a string starts with another string
 *
 * This function has been copied from lanelet2_traffic_rules/src/GenericTrafficRules.cpp as it was not exposed by the
 * Lanelet2 libraray. The function therefore has the same functionality as the GenericTrafficRules class
 *
 * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_traffic_rules/src/GenericTrafficRules.cpp
 * The source function is copyrighted under BSD 3-Clause "New" or "Revised" License a copy of that notice has been
 * included with this package
 *
 * @param str Base string
 * @param substr Starting string to check for
 *
 * @return True if str starts with substr
 */
bool startswith(const std::string& str, const std::string& substr)
{
  return str.compare(0, substr.size(), substr) == 0;
}

/**
 * @brief Helper function to determine the type of lane change implicitly allowed by the markings on the road.
 *
 * This function has been copied from lanelet2_traffic_rules/src/GenericTrafficRules.cpp as it was not exposed by the
 * Lanelet2 libraray. The function therefore has the same functionality as the GenericTrafficRules class
 *
 * https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_traffic_rules/src/GenericTrafficRules.cpp
 * The source function is copyrighted under BSD 3-Clause "New" or "Revised" License a copy of that notice has been
 * included with this package
 *
 * @param type The type of the line string marking
 * @param subtype The sub-type of the line string marking
 * @param participant The type of participant the rules apply too
 *
 * @return An enum describing the type of lane change allowed by the road markings
 */
LaneChangeType getChangeType(const std::string& type, const std::string& subtype, const std::string& participant)
{
  using LaneChangeMap = std::map<std::pair<std::string, std::string>, LaneChangeType>;
  const static LaneChangeMap VehicleChangeType{
    { { AttributeValueString::LineThin, AttributeValueString::Dashed }, LaneChangeType::Both },
    { { AttributeValueString::LineThick, AttributeValueString::Dashed }, LaneChangeType::Both },
    { { AttributeValueString::LineThin, AttributeValueString::DashedSolid }, LaneChangeType::ToRight },
    { { AttributeValueString::LineThick, AttributeValueString::DashedSolid }, LaneChangeType::ToRight },
    { { AttributeValueString::LineThin, AttributeValueString::SolidDashed }, LaneChangeType::ToLeft },
    { { AttributeValueString::LineThick, AttributeValueString::SolidDashed }, LaneChangeType::ToLeft }
  };
  const static LaneChangeMap PedestrianChangeType{ { { AttributeValueString::Curbstone, AttributeValueString::Low },
                                                     LaneChangeType::Both } };

  if (startswith(participant, Participants::Vehicle))
  {
    return getMapOrDefault(VehicleChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  if (participant == Participants::Pedestrian)
  {
    return getMapOrDefault(PedestrianChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  if (participant == Participants::Bicycle)
  {
    auto asVehicle = getMapOrDefault(VehicleChangeType, std::make_pair(type, subtype), LaneChangeType::None);
    if (asVehicle != LaneChangeType::None)
    {
      return asVehicle;
    }
    return getMapOrDefault(PedestrianChangeType, std::make_pair(type, subtype), LaneChangeType::None);
  }
  return LaneChangeType::None;
}

/**
 * @brief Helper function constructs a PassingControlLine based off the provided bound and lane change type
 *
 * @param bound The bound which will mark the control line
 * @param type The type of lane change permitted by this bound's implied regulations
 * @param participant The participant that is allowed to cross this control line
 *
 * @return A new PasssingControlLine which was created based off the bound
 */
PassingControlLinePtr buildControlLine(LineString3d& bound, const LaneChangeType type, const std::string& participant)
{
  if (bound.inverted())
  {
    switch (type)
    {
      case LaneChangeType::ToRight:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(
            PassingControlLine::buildData(lanelet::utils::getId(), { bound.invert() }, {}, { participant })));
      case LaneChangeType::ToLeft:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(
            PassingControlLine::buildData(lanelet::utils::getId(), { bound.invert() }, { participant }, {})));
      case LaneChangeType::Both:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(PassingControlLine::buildData(
            lanelet::utils::getId(), { bound.invert() }, { participant }, { participant })));
      default:  // LaneChangeType::None
        return std::shared_ptr<PassingControlLine>(
            new PassingControlLine(PassingControlLine::buildData(lanelet::utils::getId(), { bound.invert() }, {}, {})));
    }
  }
  else
  {
    switch (type)
    {
      case LaneChangeType::ToRight:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(
            PassingControlLine::buildData(lanelet::utils::getId(), { bound }, { participant }, {})));
      case LaneChangeType::ToLeft:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(
            PassingControlLine::buildData(lanelet::utils::getId(), { bound }, {}, { participant })));
      case LaneChangeType::Both:
        return std::shared_ptr<PassingControlLine>(new PassingControlLine(
            PassingControlLine::buildData(lanelet::utils::getId(), { bound }, { participant }, { participant })));
      default:  // LaneChangeType::None
        return std::shared_ptr<PassingControlLine>(
            new PassingControlLine(PassingControlLine::buildData(lanelet::utils::getId(), { bound }, {}, {})));
    }
  }
}

/**
 * @brief Generate RegionAccessRules from the inferred regulations in the provided map and lanelet
 *
 * @param lanelet The lanelet to generate the rules for
 * @param map The map which the lanelet is part of
 * @param default_traffic_rules The set of traffic rules to treat as guidance for interpreting the map
 */
void addInferredAccessRule(Lanelet& lanelet, lanelet::LaneletMapPtr map,
                           const std::vector<lanelet::traffic_rules::TrafficRulesUPtr>& default_traffic_rules)
{
  auto access_rules = lanelet.regulatoryElementsAs<RegionAccessRule>();
  // If the lanelet does not have an access rule then add one based on the generic traffic rules
  if (access_rules.size() == 0)
  {  // No access rule detected so add one

    // We want to check for all participants which are currently supported
    std::vector<std::string> allowed_participants;

    for (const auto& rules : default_traffic_rules)
    {
      if (rules->canPass(lanelet))
      {
        allowed_participants.emplace_back(rules->participant());
      }
    }

    std::shared_ptr<RegionAccessRule> rar(new RegionAccessRule(
        RegionAccessRule::buildData(lanelet::utils::getId(), { lanelet }, {}, allowed_participants)));
    lanelet.addRegulatoryElement(rar);
    map->add(rar);
  }
}

/**
 * @brief Generate RegionAccessRules from the inferred regulations in the provided map and area
 *
 * @param area The area to generate the rules for
 * @param map The map which the area is part of
 * @param default_traffic_rules The set of traffic rules to treat as guidance for interpreting the map
 */
void addInferredAccessRule(Area& area, lanelet::LaneletMapPtr map,
                           const std::vector<lanelet::traffic_rules::TrafficRulesUPtr>& default_traffic_rules)
{
  auto access_rules = area.regulatoryElementsAs<RegionAccessRule>();
  // If the lanelet does not have an access rule then add one based on the generic traffic rules
  if (access_rules.size() == 0)
  {  // No access rule detected so add one

    // We want to check for all participants which are currently supported
    std::vector<std::string> allowed_participants;

    for (const auto& rules : default_traffic_rules)
    {
      if (rules->canPass(area))
      {
        allowed_participants.emplace_back(rules->participant());
      }
    }

    std::shared_ptr<RegionAccessRule> rar(
        new RegionAccessRule(RegionAccessRule::buildData(lanelet::utils::getId(), {}, { area }, allowed_participants)));
    area.addRegulatoryElement(rar);
    map->add(rar);
  }
}

/**
 * @brief Generate PassingControlLines from the inferred regulations in the provided map and lanelet
 *
 * @param lanelet The lanelet to generate control lines for
 * @param map The map which the lanelet is part of
 */
void addInferredPassingControlLine(Lanelet& lanelet, lanelet::LaneletMapPtr map)
{
  // Since this class is only designed to add passing control lines based on lane changes
  // we will always assume the participant is a vehicle
  std::string participant(lanelet::Participants::Vehicle);

  LineString3d left_bound = lanelet.leftBound();
  LineString3d right_bound = lanelet.rightBound();

  // Determine possibility of lane change for left and right bounds
  LaneChangeType left_type = getChangeType(left_bound.attribute(AttributeName::Type).value(),
                                           left_bound.attribute(AttributeName::Subtype).value(), participant);
  LaneChangeType right_type = getChangeType(right_bound.attribute(AttributeName::Type).value(),
                                            right_bound.attribute(AttributeName::Subtype).value(), participant);

  auto local_control_lines = lanelet.regulatoryElementsAs<PassingControlLine>();

  bool foundLeft = false;
  bool foundRight = false;

  // Iterate over all regulatory elements in the map to determine if there is an existing regulation for this
  // lanelet's bounds
  for (auto reg_elem : map->regulatoryElementLayer)
  {
    if (reg_elem->attribute(AttributeName::Subtype).value() != PassingControlLine::RuleName)
    {
      continue;
    }

    auto pcl = std::static_pointer_cast<PassingControlLine>(reg_elem);
    for (auto sub_line : pcl->controlLine())
    {
      bool shouldAdd = false;
      
      if (left_bound.id() == sub_line.id() && !foundLeft)
      {
        foundLeft = true;
        shouldAdd = !lanelet::utils::contains(local_control_lines, pcl);
      }
      else if (right_bound.id() == sub_line.id() && !foundRight)
      {
        foundRight = true;
        shouldAdd = !lanelet::utils::contains(local_control_lines, pcl);
      }
      // Check if our lanelet contains this control line
      // If it does not then add it
      if (shouldAdd)
      {
        lanelet.addRegulatoryElement(pcl);
      }
    }
    
  }

  // If no existing regulation was found for this lanelet's right or left bound then create a new one and add it to
  // the lanelet and the map
  if (!foundLeft)
  {
    PassingControlLinePtr pcl_left = buildControlLine(left_bound, left_type, participant);
    lanelet.addRegulatoryElement(pcl_left);
    map->add(pcl_left);
  }
  if (!foundRight)
  {
    PassingControlLinePtr pcl_right = buildControlLine(right_bound, right_type, participant);
    lanelet.addRegulatoryElement(pcl_right);
    map->add(pcl_right);
  }
}

/**
 * @brief Generate PassingControlLines from the inferred regulations in the provided map and area
 *
 * @param area The area to generate control lines for
 * @param map The map which the area is part of
 */
void addInferredPassingControlLine(Area& area, lanelet::LaneletMapPtr map)
{
  // Since this class is only designed to add passing control lines based on lane changes
  // we will always assume the participant is a vehicle
  std::string participant(lanelet::Participants::Vehicle);

  LineStrings3d outerBounds = area.outerBound();

  auto local_control_lines = area.regulatoryElementsAs<PassingControlLine>();

  std::vector<size_t> found_indices;

  // Iterate over all regulatory elements in the map to determine if there is an existing regulation for this area's
  // bounds
  for (auto reg_elem : map->regulatoryElementLayer)
  {
    if (reg_elem->attribute(AttributeName::Subtype).value() == PassingControlLine::RuleName)
    {
      auto pcl = std::static_pointer_cast<PassingControlLine>(reg_elem);
      for (auto sub_line : pcl->controlLine())
      {
        size_t i = 0;
        for (auto sub_bound : outerBounds)
        {
          if (sub_bound.id() == sub_line.id())
          {
            found_indices.push_back(i);
            // If an existing regulation applies to this area's bounds then add it to the area
            bool alreadyAdded = lanelet::utils::contains(local_control_lines, pcl);
            if (!alreadyAdded)
            {
              area.addRegulatoryElement(pcl);
            }
          }
        }
      }
    }
  }

  // For all the bounds which did not have an existing regulation create a new one
  size_t i = 0;
  for (auto sub_bound : outerBounds)
  {
    if (lanelet::utils::contains(found_indices, i))
    {
      continue;  // Continue if this sub_bound is already accounted for
    }

    LaneChangeType change_type = getChangeType(sub_bound.attribute(AttributeName::Type).value(),
                                               sub_bound.attribute(AttributeName::Subtype).value(), participant);
    PassingControlLinePtr control_line = buildControlLine(sub_bound, change_type, participant);
    area.addRegulatoryElement(control_line);
    map->add(control_line);

    i++;
  }
}

/**
 * @brief Generate DirectionOfTravel from the inferred regulations in the provided map and lanelet
 *        Only adds a regulatory element if the direction of travel is not one_way as that is the default
 *
 * @param lanelet The lanelet to generate directions of travel for
 * @param map The map which the lanelet is part of
 * @param default_traffic_rules The set of traffic rules to treat as guidance for interpreting the map
 */
void addInferredDirectionOfTravel(Lanelet& lanelet, lanelet::LaneletMapPtr map,
                                  const std::vector<lanelet::traffic_rules::TrafficRulesUPtr>& default_traffic_rules)
{
  auto direction_of_travel = lanelet.regulatoryElementsAs<DirectionOfTravel>();
  // If the lanelet does not have an access rule then add one based on the generic traffic rules
  if (direction_of_travel.size() == 0)
  {  // No direction detected so need to check if one is required

    // We want to check for all participants which are currently supported
    std::vector<std::string> allowed_participants;

    for (const auto& rules : default_traffic_rules)
    {
      if (!rules->isOneWay(lanelet))
      {  // Check if this lanelet is not oneway
        allowed_participants.emplace_back(rules->participant());
      }
    }

    if (allowed_participants.size() > 0)
    {  // Only add bi-directional regulations
      std::shared_ptr<DirectionOfTravel> rar(new DirectionOfTravel(DirectionOfTravel::buildData(
          lanelet::utils::getId(), { lanelet }, DirectionOfTravel::BiDirectional, allowed_participants)));
      lanelet.addRegulatoryElement(rar);
      map->add(rar);
    }
  }
}

void addValidSpeedLimit(Lanelet& lanelet, lanelet::LaneletMapPtr map, lanelet::Velocity config_limit,
    const std::vector<lanelet::traffic_rules::TrafficRulesUPtr>& default_traffic_rules )
{
  lanelet::Velocity max_speed;
    auto speed_limit = lanelet.regulatoryElementsAs<DigitalSpeedLimit>();
     if(config_limit < 80_mph && config_limit > 0_mph)//Accounting for the configured speed limit, input zero when not in use
      {  
        max_speed = config_limit;
      }
      else
      {
        max_speed = 80_mph;
      }
      
    // If the lanelet does not have a digital speed limit then add one with the maximum value of 80
    std::vector<std::string> allowed_participants;
     //Maximum speed limit is 80
   

    if (speed_limit.empty())//If there is no assigned speed limit value
    {

         for(const auto& rules : default_traffic_rules)
         {
            if (rules->canPass(lanelet))
             {
              allowed_participants.emplace_back(rules->participant());
             }
        }
     if (!allowed_participants.empty())
     {

      auto rar = std::make_shared<DigitalSpeedLimit>(DigitalSpeedLimit::buildData(lanelet::utils::getId(), max_speed, {lanelet},
      {}, allowed_participants));

      lanelet.addRegulatoryElement(rar);
      map->add(rar);//Add DigitalSpeedLimit data to the map

     }
  }
  else  //If the speed limit value already exists 
  {
      for(const auto& rules : default_traffic_rules)
         {
            if (rules->canPass(lanelet))
             {
              allowed_participants.emplace_back(rules->participant());
             }
        }
    if(speed_limit.back().get()->speed_limit_ > max_speed)//Check that speed limit value does not exceed the maximum value
    {
    
      ROS_WARN_STREAM("Invalid speed limit value. Value reset to maximum speed limit.");
      auto rar = std::make_shared<DigitalSpeedLimit>(DigitalSpeedLimit::buildData(lanelet::utils::getId(), max_speed, {lanelet},
      {}, allowed_participants));
      lanelet.removeRegulatoryElement(speed_limit.back());
      lanelet.addRegulatoryElement(rar);
      map->update(lanelet, rar);//Add DigitalSpeedLimit data to the map
      ROS_INFO_STREAM("Number of Regulatory Elements: "<< map->regulatoryElementLayer.size());


    }
    
  }
}


}  // namespace

void ensureCompliance(lanelet::LaneletMapPtr map, lanelet::Velocity config_limit)
{

  auto default_traffic_rules = getAllGermanTrafficRules();  // Use german traffic rules as default as they most closely
                                                            // match the generic traffic rules
  // Handle lanelets
  for (auto lanelet : map->laneletLayer)
  {
    addInferredAccessRule(lanelet, map, default_traffic_rules);
    addInferredPassingControlLine(lanelet, map);
    addInferredDirectionOfTravel(lanelet, map, default_traffic_rules);
      addValidSpeedLimit(lanelet, map, config_limit, default_traffic_rules);// 0_mph can be changed with the config_limit


  }
  // Handle areas
  for (auto area : map->areaLayer)
  {
    addInferredAccessRule(area, map, default_traffic_rules);
    addInferredPassingControlLine(area, map);
  }
}
};  // namespace MapConformer
}  // namespace lanelet