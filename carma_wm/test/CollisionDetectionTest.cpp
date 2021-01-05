/*
 * Copyright (C) 2019-2021 LEIDOS.dev
 * 
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
#include <carma_wm/Geometry.h>
#include <carma_wm/collision_detection.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/Attribute.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "TestHelpers.h"

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm
{

  TEST(CollisionDetectionTest, CheckPolygonIntersection)
  {

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 0;

    collision_detection::polygon_t ob1, ob2;

    boost::geometry::read_wkt(
    "POLYGON((2 1.0, 2.0 2.0, 3.0 2.0, 3.0 1.0))", ob1);

    boost::geometry::read_wkt(
    "POLYGON((1.0 1.0, 3.0 1.0, 2.0 1.0, 3.0 2.0))", ob2);

    collision_detection::MovingObject mo1 = {ob1, linear_velocity};
    collision_detection::MovingObject mo2 = {ob2, linear_velocity};

    ASSERT_TRUE(collision_detection::CheckPolygonIntersection(mo1, mo2));

    boost::geometry::clear(ob1);
    boost::geometry::clear(ob2);

    boost::geometry::read_wkt(
    "POLYGON((0 0, 1 0, 1 1, 0 1))", ob1);

    boost::geometry::read_wkt(
    "POLYGON((10 10, 10 20, 20 20, 20 10))", ob2);

    mo1 = {ob1, linear_velocity};
    mo2 = {ob2, linear_velocity};

    ASSERT_FALSE(collision_detection::CheckPolygonIntersection(mo1, mo2));
  }


  TEST(CollisionDetectionTest, ObjectToBoostPolygon)
  {
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

    collision_detection::polygon_t result = collision_detection::ObjectToBoostPolygon<collision_detection::polygon_t>(pose, size);

    std::vector<collision_detection::point_t> points = result.outer();

    ASSERT_NEAR(points.at(0).get<0>(), 5.0, 0.00001);
    ASSERT_NEAR(points.at(0).get<1>(), 7.0, 0.00001);

    ASSERT_NEAR(points.at(1).get<0>(), 7.0, 0.00001);
    ASSERT_NEAR(points.at(1).get<1>(), 7.0, 0.00001);

    ASSERT_NEAR(points.at(2).get<0>(), 7.0, 0.00001);
    ASSERT_NEAR(points.at(2).get<1>(), 3.0, 0.00001);

    ASSERT_NEAR(points.at(3).get<0>(), 5.0, 0.00001);
    ASSERT_NEAR(points.at(3).get<1>(), 3.0, 0.00001);
  }


  TEST(CollisionDetectionTest, PredictObjectPosition)
  {

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 0;

    collision_detection::polygon_t ob1;
    collision_detection::polygon_t ob2;
    collision_detection::polygon_t ob3;
    collision_detection::polygon_t ob4;

    boost::geometry::read_wkt(
    "POLYGON((1.0 1.0, 1.0 2.0, 2.0 2.0, 2.0 1.0))", ob1);

    boost::geometry::read_wkt(
    "POLYGON((2.0 1.0, 2.0 2.0, 3.0 2.0, 3.0 1.0))", ob2);

    boost::geometry::read_wkt(
    "POLYGON((3.0 1.0, 3.0 2.0, 4.0 2.0, 4.0 1.0))", ob3);

    boost::geometry::read_wkt(
    "POLYGON((4.0 1.0, 4.0 2.0, 5.0 2.0, 5.0 1.0))", ob4);


    std::vector<collision_detection::polygon_t> future_polygons;

    future_polygons.push_back(ob2);
    future_polygons.push_back(ob3);
    future_polygons.push_back(ob4);

    std::tuple <__uint64_t,collision_detection::polygon_t> fo1(1.0,ob2);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo2(2.0,ob3);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo3(3.0,ob4);

    std::vector<std::tuple <__uint64_t,collision_detection::polygon_t>> future_polygons_tuple;
    future_polygons_tuple.push_back(fo1);
    future_polygons_tuple.push_back(fo2);
    future_polygons_tuple.push_back(fo3);


    collision_detection::MovingObject mo1 = {ob1, linear_velocity, future_polygons_tuple};

    __uint64_t target_time = 3;
    
    collision_detection::MovingObject result = collision_detection::PredictObjectPosition(mo1,target_time);

    std::vector<collision_detection::point_t> points = result.object_polygon.outer();

    ASSERT_NEAR(points.at(0).get<0>(), 2.0, 0.00001);
    ASSERT_NEAR(points.at(0).get<1>(), 1.0, 0.00001);

    ASSERT_NEAR(points.at(1).get<0>(), 2.0, 0.00001);
    ASSERT_NEAR(points.at(1).get<1>(), 2.0, 0.00001);

    ASSERT_NEAR(points.at(2).get<0>(), 5.0, 0.00001);
    ASSERT_NEAR(points.at(2).get<1>(), 2.0, 0.00001);

    ASSERT_NEAR(points.at(3).get<0>(), 5.0, 0.00001);
    ASSERT_NEAR(points.at(3).get<1>(), 1.0, 0.00001);

    ASSERT_NEAR(points.at(4).get<0>(), 2.0, 0.00001);
    ASSERT_NEAR(points.at(4).get<1>(), 1.0, 0.00001);

  }


  TEST(CollisionDetectionTest, DetectCollision)
  {

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 0;

    collision_detection::polygon_t ob1;
    collision_detection::polygon_t ob2;
    collision_detection::polygon_t ob3;
    collision_detection::polygon_t ob4;

    boost::geometry::read_wkt(
    "POLYGON((1.0 1.0, 1.0 2.0, 2.0 2.0, 2.0 1.0))", ob1);

    boost::geometry::read_wkt(
    "POLYGON((2.0 1.0, 2.0 2.0, 3.0 2.0, 3.0 1.0))", ob2);

    boost::geometry::read_wkt(
    "POLYGON((3.0 1.0, 3.0 2.0, 4.0 2.0, 4.0 1.0))", ob3);

    boost::geometry::read_wkt(
    "POLYGON((4.0 1.0, 4.0 2.0, 5.0 2.0, 5.0 1.0))", ob4);


    std::vector<collision_detection::polygon_t> future_polygons_1;

    future_polygons_1.push_back(ob2);
    future_polygons_1.push_back(ob3);
    future_polygons_1.push_back(ob4);


    std::tuple <__uint64_t,collision_detection::polygon_t> fo1(1.0,ob2);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo2(2.0,ob3);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo3(3.0,ob4);

    std::vector<std::tuple <__uint64_t,collision_detection::polygon_t>> future_polygons_tuple_1;
    future_polygons_tuple_1.push_back(fo1);
    future_polygons_tuple_1.push_back(fo2);
    future_polygons_tuple_1.push_back(fo3);


    collision_detection::MovingObject mo1 = {ob1, linear_velocity, future_polygons_tuple_1};

    collision_detection::polygon_t ob5;
    collision_detection::polygon_t ob6;
    collision_detection::polygon_t ob7;
    collision_detection::polygon_t ob8;


    boost::geometry::read_wkt(
    "POLYGON((1.0 1.0, 1.0 2.0, 2.0 2.0, 2.0 1.0))", ob5);

    boost::geometry::read_wkt(
    "POLYGON((2.0 1.0, 2.0 2.0, 3.0 2.0, 3.0 1.0))", ob6);

    boost::geometry::read_wkt(
    "POLYGON((3.0 1.0, 3.0 2.0, 4.0 2.0, 4.0 1.0))", ob7);

    boost::geometry::read_wkt(
    "POLYGON((4.0 1.0, 4.0 2.0, 5.0 2.0, 5.0 1.0))", ob8);


    std::vector<collision_detection::polygon_t> future_polygons_2;

    future_polygons_2.push_back(ob6);
    future_polygons_2.push_back(ob7);
    future_polygons_2.push_back(ob8);

    std::tuple <__uint64_t,collision_detection::polygon_t> fo6(1.0,ob6);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo7(2.0,ob7);
    std::tuple <__uint64_t,collision_detection::polygon_t> fo8(3.0,ob8);

    std::vector<std::tuple <__uint64_t,collision_detection::polygon_t>> future_polygons_tuple_2;
    future_polygons_tuple_2.push_back(fo6);
    future_polygons_tuple_2.push_back(fo7);
    future_polygons_tuple_2.push_back(fo8);

    collision_detection::MovingObject mo2 = {ob5, linear_velocity, future_polygons_tuple_2};


    bool result = collision_detection::DetectCollision(mo1, mo2,3);

    ASSERT_TRUE(result);

  }

  TEST(CollisionDetectionTest, WorldCollisionDetection)
  {

    cav_msgs::RoadwayObstacleList rwol;
    cav_msgs::TrajectoryPlan tp;

    
    geometry_msgs::Twist veloctiy;
    __uint64_t target_time = 3;

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 0;

    veloctiy.linear = linear_velocity;

    
    geometry_msgs::Vector3 size;
    size.x = 4;
    size.y = 2;
    size.z = 1;


    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.target_time = ros::Time(0, 0);

    trajectory_point_2.x = 1.0;
    trajectory_point_2.y = 2.0;
    trajectory_point_2.target_time = ros::Time(0, 1);

    trajectory_point_3.x = 1.0;
    trajectory_point_3.y = 3.0;
    trajectory_point_3.target_time = ros::Time(0, 2);

    trajectory_point_4.x = 1.0;
    trajectory_point_4.y = 4.0;
    trajectory_point_4.target_time = ros::Time(0, 3);

    trajectory_point_5.x = 1.0;
    trajectory_point_5.y = 5.0;
    trajectory_point_5.target_time = ros::Time(0, 4);

    tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5};

    cav_msgs::RoadwayObstacle rwo_1;
    cav_msgs::RoadwayObstacle rwo_2;
    cav_msgs::RoadwayObstacle rwo_3;
    cav_msgs::RoadwayObstacle rwo_4;
    cav_msgs::RoadwayObstacle rwo_5;

    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(0, 0, 1.5708);


    rwo_1.object.pose.pose.position.x = 6;
    rwo_1.object.pose.pose.position.y = 5;
    rwo_1.object.pose.pose.position.z = 0;


    rwo_1.object.pose.pose.orientation.x = tf_orientation.getX();
    rwo_1.object.pose.pose.orientation.y = tf_orientation.getY();
    rwo_1.object.pose.pose.orientation.z = tf_orientation.getZ();
    rwo_1.object.pose.pose.orientation.w = tf_orientation.getW();


    rwo_1.object.size.x = 1;
    rwo_1.object.size.y = 1;
    rwo_1.object.size.z = 1;

    cav_msgs::PredictedState ps_1;
    ps_1.header.stamp.nsec = 1000;

    ps_1.predicted_position.position.x = 1;
    ps_1.predicted_position.position.y = 1;
    ps_1.predicted_position.position.z = 0;

    ps_1.predicted_position.orientation.x = tf_orientation.getX();
    ps_1.predicted_position.orientation.y = tf_orientation.getY();
    ps_1.predicted_position.orientation.z = tf_orientation.getZ();
    ps_1.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_2;
    ps_2.header.stamp.nsec = 2000;

    ps_2.predicted_position.position.x = 1;
    ps_2.predicted_position.position.y = 2;
    ps_2.predicted_position.position.z = 0;

    ps_2.predicted_position.orientation.x = tf_orientation.getX();
    ps_2.predicted_position.orientation.y = tf_orientation.getY();
    ps_2.predicted_position.orientation.z = tf_orientation.getZ();
    ps_2.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_3;
    ps_3.header.stamp.nsec = 3000;

    ps_3.predicted_position.position.x = 1;
    ps_3.predicted_position.position.y = 3;
    ps_3.predicted_position.position.z = 0;

    ps_3.predicted_position.orientation.x = tf_orientation.getX();
    ps_3.predicted_position.orientation.y = tf_orientation.getY();
    ps_3.predicted_position.orientation.z = tf_orientation.getZ();
    ps_3.predicted_position.orientation.w = tf_orientation.getW();

    rwo_1.object.predictions = {ps_1,ps_2,ps_3};

    rwol.roadway_obstacles = {rwo_1};

    std::vector<cav_msgs::RoadwayObstacle> result = collision_detection::WorldCollisionDetection(rwol, tp, size, veloctiy, target_time);

    ASSERT_EQ(result.size(),1);

  }

  TEST(CollisionDetectionFalseTest, WorldCollisionDetection)
  {

    cav_msgs::RoadwayObstacleList rwol;
    cav_msgs::TrajectoryPlan tp;

    geometry_msgs::Twist veloctiy;
    const __uint64_t target_time = 3;

    geometry_msgs::Vector3 linear_velocity;
    linear_velocity.x = 0;
    linear_velocity.y = 1;

    veloctiy.linear = linear_velocity;

    geometry_msgs::Vector3 size;
    size.x = 1;
    size.y = 2;
    size.z = 1;

    cav_msgs::TrajectoryPlanPoint trajectory_point_1;
    cav_msgs::TrajectoryPlanPoint trajectory_point_2;
    cav_msgs::TrajectoryPlanPoint trajectory_point_3;
    cav_msgs::TrajectoryPlanPoint trajectory_point_4;
    cav_msgs::TrajectoryPlanPoint trajectory_point_5;

    trajectory_point_1.x = 1.0;
    trajectory_point_1.y = 1.0;
    trajectory_point_1.target_time = ros::Time(0, 0);

    trajectory_point_2.x = 1.0;
    trajectory_point_2.y = 2.0;
    trajectory_point_2.target_time = ros::Time(0, 1);

    trajectory_point_3.x = 1.0;
    trajectory_point_3.y = 3.0;
    trajectory_point_3.target_time = ros::Time(0, 2);

    trajectory_point_4.x = 1.0;
    trajectory_point_4.y = 4.0;
    trajectory_point_4.target_time = ros::Time(0, 3);

    trajectory_point_5.x = 1.0;
    trajectory_point_5.y = 5.0;
    trajectory_point_5.target_time = ros::Time(0, 4);

    tp.trajectory_points = {trajectory_point_1, trajectory_point_2, trajectory_point_3, trajectory_point_4, trajectory_point_5};

    cav_msgs::RoadwayObstacle rwo_1;
    cav_msgs::RoadwayObstacle rwo_2;
    cav_msgs::RoadwayObstacle rwo_3;
    cav_msgs::RoadwayObstacle rwo_4;
    cav_msgs::RoadwayObstacle rwo_5;

    tf2::Quaternion tf_orientation;
    tf_orientation.setRPY(0, 0, 1.5708);
    rwo_1.object.pose.pose.position.x = 6;
    rwo_1.object.pose.pose.position.y = 5;
    rwo_1.object.pose.pose.position.z = 0;

    rwo_1.object.pose.pose.orientation.x = tf_orientation.getX();
    rwo_1.object.pose.pose.orientation.y = tf_orientation.getY();
    rwo_1.object.pose.pose.orientation.z = tf_orientation.getZ();
    rwo_1.object.pose.pose.orientation.w = tf_orientation.getW();

    rwo_1.object.size.x = 1;
    rwo_1.object.size.y = 1;
    rwo_1.object.size.z = 1;

    cav_msgs::PredictedState ps_1;
    ps_1.header.stamp.nsec = 1000;

    ps_1.predicted_position.position.x = 7;
    ps_1.predicted_position.position.y = 8;
    ps_1.predicted_position.position.z = 0;

    ps_1.predicted_position.orientation.x = tf_orientation.getX();
    ps_1.predicted_position.orientation.y = tf_orientation.getY();
    ps_1.predicted_position.orientation.z = tf_orientation.getZ();
    ps_1.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_2;
    ps_2.header.stamp.nsec = 2000;

    ps_2.predicted_position.position.x = 9;
    ps_2.predicted_position.position.y = 10;
    ps_2.predicted_position.position.z = 0;

    ps_2.predicted_position.orientation.x = tf_orientation.getX();
    ps_2.predicted_position.orientation.y = tf_orientation.getY();
    ps_2.predicted_position.orientation.z = tf_orientation.getZ();
    ps_2.predicted_position.orientation.w = tf_orientation.getW();

    cav_msgs::PredictedState ps_3;
    ps_3.header.stamp.nsec = 3000;

    ps_3.predicted_position.position.x = 11;
    ps_3.predicted_position.position.y = 12;
    ps_3.predicted_position.position.z = 0;

    ps_3.predicted_position.orientation.x = tf_orientation.getX();
    ps_3.predicted_position.orientation.y = tf_orientation.getY();
    ps_3.predicted_position.orientation.z = tf_orientation.getZ();
    ps_3.predicted_position.orientation.w = tf_orientation.getW();

    rwo_1.object.predictions = {ps_1,ps_2,ps_3};

    rwol.roadway_obstacles = {rwo_1};

    std::vector<cav_msgs::RoadwayObstacle> result = collision_detection::WorldCollisionDetection(rwol, tp, size, veloctiy, target_time);

    ASSERT_EQ(result.size(),0);

  }

}  // namespace carma_wm
