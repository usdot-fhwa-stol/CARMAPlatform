#pragma once
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

#include <gmock/gmock.h>
#include <iostream>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_core/Attribute.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <thread>

/**
 * Helper file containing inline functions used to quickly build lanelet objects in unit tests
 *
 */
namespace carma_wm
{
inline lanelet::Point3d getPoint(double x, double y, double z)
{
  return lanelet::Point3d(lanelet::utils::getId(), x, y, z);
}

inline lanelet::BasicPoint3d getBasicPoint(double x, double y, double z)
{
  return getPoint(x, y, z).basicPoint();
}

inline lanelet::BasicPoint2d getBasicPoint(double x, double y)
{
  return lanelet::utils::to2D(getPoint(x, y, 0.0)).basicPoint();
}

// Defaults to double solid line on left and double solid line on right
inline lanelet::Lanelet getLanelet(lanelet::Id id, lanelet::LineString3d& left_ls, lanelet::LineString3d& right_ls,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  left_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  left_ls.attributes()[lanelet::AttributeName::Subtype] = left_sub_type;

  right_ls.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  right_ls.attributes()[lanelet::AttributeName::Subtype] = right_sub_type;

  lanelet::Lanelet ll;
  ll.setId(id);
  ll.setLeftBound(left_ls);
  ll.setRightBound(right_ls);

  ll.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Lanelet;
  ll.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  ll.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  ll.attributes()[lanelet::AttributeName::OneWay] = "yes";
  ll.attributes()[lanelet::AttributeName::Dynamic] = "no";
  ll.attributes()[lanelet::AttributeNamesString::ParticipantVehicle] = "yes";

  return ll;
}

inline lanelet::Lanelet getLanelet(lanelet::Id id, std::vector<lanelet::Point3d> left,
                                   std::vector<lanelet::Point3d> right,
                                   const lanelet::Attribute& left_sub_type = lanelet::AttributeValueString::SolidSolid,
                                   const lanelet::Attribute& right_sub_type = lanelet::AttributeValueString::Solid)
{
  lanelet::LineString3d left_ls(lanelet::utils::getId(), left);

  lanelet::LineString3d right_ls(lanelet::utils::getId(), right);

  return getLanelet(id, left_ls, right_ls, left_sub_type, right_sub_type);
}

inline lanelet::LaneletMapPtr getDisjointRouteMap()
{
  // 1. Construct map
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 2, 0);
  auto p5 = getPoint(1, 0, 0);
  auto p6 = getPoint(2, 0, 0);
  auto p7 = getPoint(2, 1, 0);
  auto p8 = getPoint(2, 2, 0);
  auto p9 = getPoint(1, 3, 0);
  auto p10 = getPoint(2, 3, 0);
  auto p11 = getPoint(1, 4, 0);  // Points for areas
  auto p12 = getPoint(2, 4, 0);
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p5, p3 });
  auto ll_1 = getLanelet(10000, left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  lanelet::LineString3d right_ls_2(lanelet::utils::getId(), { p6, p7 });
  auto ll_2 = getLanelet(10001, right_ls_1, right_ls_2, lanelet::AttributeValueString::Dashed,
                         lanelet::AttributeValueString::Solid);

  lanelet::LineString3d left_ls_3(lanelet::utils::getId(), { p3, p4 });
  lanelet::LineString3d right_ls_3(lanelet::utils::getId(), { p7, p8 });
  auto ll_3 = getLanelet(10002, left_ls_3, right_ls_3, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);

  // Add two way linestring
  lanelet::LineString3d left_ls_4(lanelet::utils::getId(), { p4, p9 });
  lanelet::LineString3d right_ls_4(lanelet::utils::getId(), { p8, p10 });
  auto ll_4 = getLanelet(10003, left_ls_4, right_ls_4, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);
  ll_4.attributes()[lanelet::AttributeName::OneWay] = "no";

  // Add an area
  lanelet::LineString3d area_loop(lanelet::utils::getId(), { p9, p11, p12, p10 });

  area_loop.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  area_loop.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;

  lanelet::Area area(10004, { area_loop });

  area.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Multipolygon;
  area.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  area.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  area.attributes()[lanelet::AttributeNamesString::ParticipantVehicle] = "yes";

  // Create basic map
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2, ll_3, ll_4 }, { area });

  return map;
}


inline lanelet::LaneletMapPtr getBroadcasterTestMap()
{
  using namespace carma_wm;
  // 1. Construct map
  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(1, 1, 0);
  auto p4 = getPoint(1, 2, 0);
  auto p5 = getPoint(1, 0, 0);
  auto p6 = getPoint(2, 0, 0);
  auto p7 = getPoint(2, 1, 0);
  auto p8 = getPoint(2, 2, 0);
  auto p9 = getPoint(1, 3, 0);
  auto p10 = getPoint(2, 3, 0);
  auto p11 = getPoint(1, 4, 0);  // Points for areas
  auto p12 = getPoint(2, 4, 0);
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p5, p3 });
  auto ll_1 = getLanelet(10000, left_ls_1, right_ls_1, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  lanelet::LineString3d right_ls_2(lanelet::utils::getId(), { p6, p7 });
  auto ll_2 = getLanelet(10001, right_ls_1, right_ls_2, lanelet::AttributeValueString::Dashed,
                         lanelet::AttributeValueString::Solid);

  lanelet::LineString3d left_ls_3(lanelet::utils::getId(), { p3, p4 });
  lanelet::LineString3d right_ls_3(lanelet::utils::getId(), { p7, p8 });
  auto ll_3 = getLanelet(10002, left_ls_3, right_ls_3, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);

  // Add two way linestring
  lanelet::LineString3d left_ls_4(lanelet::utils::getId(), { p4, p9 });
  lanelet::LineString3d right_ls_4(lanelet::utils::getId(), { p8, p10 });
  auto ll_4 = getLanelet(10003, left_ls_4, right_ls_4, lanelet::AttributeValueString::Solid,
                         lanelet::AttributeValueString::Solid);
  ll_4.attributes()[lanelet::AttributeName::OneWay] = "no";

  // We will modify the map here to include 
  // opposite lanelet for id 10000
  lanelet::LineString3d left_ls_inv(lanelet::utils::getId(), { p2, p1 });
  lanelet::LineString3d right_ls_inv(lanelet::utils::getId(), { p3, p5 });
  auto ll_1_inv = carma_wm::getLanelet(10005, left_ls_inv, right_ls_inv, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  // 2 different direction lanelets -> |/ (directions)
  // both successors of id 10000
  auto p13 = carma_wm::getPoint(0, 2, 0);
  lanelet::LineString3d left_ls_5(lanelet::utils::getId(), { p2, p4 });
  lanelet::LineString3d right_ls_5(lanelet::utils::getId(), { p3, p8 });
  auto ll_6 = carma_wm::getLanelet(10006, left_ls_5, right_ls_5, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);
  lanelet::LineString3d left_ls_6(lanelet::utils::getId(), { p2, p13 });
  auto ll_5 = carma_wm::getLanelet(10007, left_ls_6, left_ls_3, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  // Add an area
  lanelet::LineString3d area_loop(lanelet::utils::getId(), { p9, p11, p12, p10 });

  area_loop.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::LineThin;
  area_loop.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Dashed;

  lanelet::Area area(10004, { area_loop });

  area.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::Multipolygon;
  area.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  area.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  area.attributes()[lanelet::AttributeNamesString::ParticipantVehicle] = "yes";

  // Create basic map
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1, ll_2, ll_3, ll_4 }, { area });

  // create disjoint lanelet, created last to not affect other ids
  auto p1_unreg = getPoint(0, 1, 0); // overlapping lanelet pointing to right, perpendicular direction from others
  auto p2_unreg = getPoint(0, 2, 0); 
  auto p3_unreg = getPoint(2, 1, 0); // notice that this lanelet is long, not connecting to any of those in right lane
  auto p4_unreg = getPoint(2, 2, 0);
  lanelet::LineString3d right_ls_unreg(lanelet::utils::getId(), { p1_unreg, p3_unreg });
  lanelet::LineString3d left_ls_unreg(lanelet::utils::getId(), { p2_unreg, p4_unreg });
  auto ll_unreg = getLanelet(10008, left_ls_unreg, right_ls_unreg, lanelet::AttributeValueString::SolidSolid,
                         lanelet::AttributeValueString::Dashed);

  map->add(ll_1_inv);
  map->add(ll_6);
  map->add(ll_5);
  map->add(ll_unreg);
  return map;
}

}  // namespace carma_wm