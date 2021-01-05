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

#include <roadway_objects/RoadwayObjectsWorker.h>
#include <carma_wm/CARMAWorldModel.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <gtest/gtest.h>
#include "TestHelpers.h"

namespace objects
{
TEST(RoadwayObjectsWorkerTest, testExternalObjectCallback)
{
  std::shared_ptr<carma_wm::CARMAWorldModel> cmw = std::make_shared<carma_wm::CARMAWorldModel>();

  // Build map
  auto p1 = carma_wm::getPoint(9, 0, 0);
  auto p2 = carma_wm::getPoint(9, 9, 0);
  auto p3 = carma_wm::getPoint(2, 0, 0);
  auto p4 = carma_wm::getPoint(2, 9, 0);
  lanelet::LineString3d right_ls_1(lanelet::utils::getId(), { p1, p2 });
  lanelet::LineString3d left_ls_1(lanelet::utils::getId(), { p3, p4 });
  auto ll_1 = carma_wm::getLanelet(left_ls_1, right_ls_1);
  lanelet::LaneletMapPtr map = lanelet::utils::createMap({ ll_1 }, {});

  // Build external object
  geometry_msgs::Pose pose;
  pose.position.x = 6;
  pose.position.y = 5;
  pose.position.z = 0;

  tf2::Quaternion tf_orientation;
  tf_orientation.setRPY(0, 0, 1.5708);

  pose.orientation.x = tf_orientation.getX();
  pose.orientation.y = tf_orientation.getY();
  pose.orientation.z = tf_orientation.getZ();
  pose.orientation.w = tf_orientation.getW();

  geometry_msgs::Vector3 size;
  size.x = 4;
  size.y = 2;
  size.z = 1;

  cav_msgs::ExternalObject obj;
  obj.id = 1;
  obj.object_type = cav_msgs::ExternalObject::SMALL_VEHICLE;
  obj.pose.pose = pose;
  obj.velocity.twist.linear.x = 1.0;

  cav_msgs::PredictedState pred;
  auto pred_pose = obj.pose.pose;
  pred_pose.position.y += 1;
  pred.predicted_position = pred_pose;
  pred.predicted_position_confidence = 1.0;

  obj.predictions.push_back(pred);

  // Build roadway objects worker to test
  cav_msgs::RoadwayObstacleList resulting_objs;

  RoadwayObjectsWorker row(std::static_pointer_cast<const carma_wm::WorldModel>(cmw),
                           [&](const cav_msgs::RoadwayObstacleList& objs) -> void { resulting_objs = objs; });

  ASSERT_EQ(resulting_objs.roadway_obstacles.size(), 0);  // Verify resulting_objs is empty

  // Build external object list
  cav_msgs::ExternalObjectList obj_list;
  obj_list.objects.push_back(obj);
  cav_msgs::ExternalObjectListConstPtr obj_list_msg_ptr(new cav_msgs::ExternalObjectList(obj_list));

  // Test with no map set
  row.externalObjectsCallback(obj_list_msg_ptr);  // Call function under test

  ASSERT_EQ(resulting_objs.roadway_obstacles.size(), 0);

  // Test with empty map
  lanelet::LaneletMapPtr empty_map = lanelet::utils::createMap({}, {});

  cmw->setMap(empty_map);

  row.externalObjectsCallback(obj_list_msg_ptr);  // Call function under test

  ASSERT_EQ(resulting_objs.roadway_obstacles.size(), 0);

  // Test with regular map
  cmw->setMap(map);

  row.externalObjectsCallback(obj_list_msg_ptr);  // Call function under test

  ASSERT_EQ(resulting_objs.roadway_obstacles.size(), 1);

  cav_msgs::RoadwayObstacle obs = resulting_objs.roadway_obstacles[0];

  ASSERT_EQ(obs.object.id, obj.id);  // Check that the object was coppied

  ASSERT_EQ(obs.lanelet_id, ll_1.id());

  ASSERT_EQ(obs.connected_vehicle_type.type, cav_msgs::ConnectedVehicleType::NOT_CONNECTED);

  ASSERT_NEAR(obs.cross_track, 0.5, 0.00001);

  ASSERT_NEAR(obs.down_track, 5.0, 0.00001);

  ASSERT_EQ(obs.predicted_lanelet_ids.size(), 1);
  ASSERT_EQ(obs.predicted_lanelet_ids[0], ll_1.id());

  ASSERT_EQ(obs.predicted_lanelet_id_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_lanelet_id_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_cross_tracks.size(), 1);
  ASSERT_NEAR(obs.predicted_cross_tracks[0], 0.5, 0.00001);

  ASSERT_EQ(obs.predicted_cross_track_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_cross_track_confidences[0], 0.9, 0.00001);

  ASSERT_EQ(obs.predicted_down_tracks.size(), 1);
  ASSERT_NEAR(obs.predicted_down_tracks[0], 6.0, 0.00001);

  ASSERT_EQ(obs.predicted_down_track_confidences.size(), 1);
  ASSERT_NEAR(obs.predicted_down_track_confidences[0], 0.9, 0.00001);
}

}  // namespace objects
