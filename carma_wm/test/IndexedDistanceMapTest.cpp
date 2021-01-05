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

#include <gmock/gmock.h>
#include <iostream>
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/geometry/LineString.h>
#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{
TEST(IndexedDistanceMapTest, IndexedDistanceMap)
{
  IndexedDistanceMap map;

  auto p1 = getPoint(0, 0, 0);
  auto p2 = getPoint(0, 1, 0);
  auto p3 = getPoint(0, 2, 0);
  auto p4 = getPoint(0, 3, 0);

  auto p5 = getPoint(0, 4, 0);
  auto p6 = getPoint(0, 5, 0);

  auto p7 = getPoint(0, 6, 0);
  auto p8 = getPoint(0, 7, 0);

  lanelet::LineString3d ls_1(lanelet::utils::getId(), { p1, p2, p3, p4 });
  lanelet::LineString3d ls_2(lanelet::utils::getId(), { p5, p6 });
  lanelet::LineString3d ls_3(lanelet::utils::getId(), { p7, p8 });

  // Check initial sizes are 0
  ASSERT_EQ(0, map.totalLength());
  ASSERT_EQ(0, map.size());

  // Add lines
  map.pushBack(lanelet::utils::to2D(ls_1));
  map.pushBack(lanelet::utils::to2D(ls_2));
  map.pushBack(lanelet::utils::to2D(ls_3));

  ASSERT_EQ(3, map.size());
  ASSERT_NEAR(5, map.totalLength(), 0.0000001);
  ASSERT_EQ(4, map.size(0));
  ASSERT_EQ(2, map.size(1));
  ASSERT_EQ(2, map.size(2));

  // Check exception on duplicate push
  ASSERT_THROW(map.pushBack(lanelet::utils::to2D(ls_1)), std::invalid_argument);

  // Check element length
  ASSERT_NEAR(3.0, map.elementLength(0), 0.000000001);
  ASSERT_NEAR(1.0, map.elementLength(1), 0.000000001);
  ASSERT_NEAR(1.0, map.elementLength(2), 0.000000001);

  // Check distance to element
  ASSERT_NEAR(0.0, map.distanceToElement(0), 0.000000001);
  ASSERT_NEAR(3.0, map.distanceToElement(1), 0.000000001);
  ASSERT_NEAR(4.0, map.distanceToElement(2), 0.000000001);

  // Check distance to point along element
  ASSERT_NEAR(0.0, map.distanceToPointAlongElement(0, 0), 0.000000001);
  ASSERT_NEAR(3.0, map.distanceToPointAlongElement(0, 3), 0.000000001);
  ASSERT_NEAR(1.0, map.distanceToPointAlongElement(1, 1), 0.000000001);

  // Check distance between
  ASSERT_NEAR(0.0, map.distanceBetween(0, 0, 0), 0.000000001);
  ASSERT_NEAR(1.0, map.distanceBetween(0, 0, 1), 0.000000001);
  ASSERT_NEAR(2.0, map.distanceBetween(0, 0, 2), 0.000000001);
  ASSERT_NEAR(3.0, map.distanceBetween(0, 0, 3), 0.000000001);
  ASSERT_NEAR(2.0, map.distanceBetween(0, 1, 3), 0.000000001);
  ASSERT_NEAR(1.0, map.distanceBetween(0, 2, 3), 0.000000001);
  ASSERT_NEAR(2.0, map.distanceBetween(0, 3, 1), 0.000000001);
  ASSERT_NEAR(1.0, map.distanceBetween(1, 0, 1), 0.000000001);

  // Check getIndexFromId
  ASSERT_EQ(0, map.getIndexFromId(ls_1.id()).first);
  ASSERT_EQ(1, map.getIndexFromId(ls_2.id()).first);
  ASSERT_EQ(2, map.getIndexFromId(ls_3.id()).first);

  ASSERT_EQ(0, map.getIndexFromId(p1.id()).first);
  ASSERT_EQ(0, map.getIndexFromId(p1.id()).second);

  ASSERT_EQ(0, map.getIndexFromId(p2.id()).first);
  ASSERT_EQ(1, map.getIndexFromId(p2.id()).second);

  ASSERT_EQ(0, map.getIndexFromId(p4.id()).first);
  ASSERT_EQ(3, map.getIndexFromId(p4.id()).second);

  ASSERT_EQ(2, map.getIndexFromId(p8.id()).first);
  ASSERT_EQ(1, map.getIndexFromId(p8.id()).second);
}
}  // namespace carma_wm
