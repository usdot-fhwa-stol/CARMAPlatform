
/*------------------------------------------------------------------------------
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

------------------------------------------------------------------------------*/

#include "carma_wm/collision_detection.h"

namespace carma_wm {

    namespace collision_detection {

        std::vector<cav_msgs::RoadwayObstacle> WorldCollisionDetection(const cav_msgs::RoadwayObstacleList& rwol, const cav_msgs::TrajectoryPlan& tp, const geometry_msgs::Vector3& size, const geometry_msgs::Twist& veloctiy, const __uint64_t target_time){

            std::vector<cav_msgs::RoadwayObstacle> rwo_collison;

            collision_detection::MovingObject vehicle_object = ConvertVehicleToMovingObject(tp, size, veloctiy);

            for (auto i : rwol.roadway_obstacles){

                collision_detection::MovingObject rwo = ConvertRoadwayObstacleToMovingObject(i);

                bool collision = DetectCollision(vehicle_object, rwo, target_time);

                if(collision) {
                    rwo_collison.push_back(i);
                }
            }

            return rwo_collison;
        };

        collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(const cav_msgs::RoadwayObstacle& rwo){

            collision_detection::MovingObject mo;
            
            mo.object_polygon = ObjectToBoostPolygon<polygon_t>(rwo.object.pose.pose, rwo.object.size);

            // Add future polygons for roadway obstacle
            for (auto i : rwo.object.predictions){
                std::tuple <__uint64_t,polygon_t> future_object(i.header.stamp.toNSec() / 1000000,ObjectToBoostPolygon<polygon_t>(i.predicted_position, rwo.object.size));
                
                mo.fp.push_back(future_object);
            }

            mo.linear_velocity = rwo.object.velocity.twist.linear;

            return mo;
        };

        collision_detection::MovingObject ConvertVehicleToMovingObject(const cav_msgs::TrajectoryPlan& tp, const geometry_msgs::Vector3& size, const geometry_msgs::Twist& veloctiy){
            
            collision_detection::MovingObject v;

            geometry_msgs::Pose pose;
            pose.position.x = tp.trajectory_points[0].x;
            pose.position.y = tp.trajectory_points[0].y;

            Eigen::Vector2d vehicle_vector = {tp.trajectory_points[1].x - tp.trajectory_points[0].x , tp.trajectory_points[1].y - tp.trajectory_points[0].y};

            Eigen::Vector2d x_axis = {1, 0};

            double yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));

            tf2::Quaternion vehicle_orientation;
            vehicle_orientation.setRPY(0, 0, yaw);

            pose.orientation.x = vehicle_orientation.getX();
            pose.orientation.y = vehicle_orientation.getY();
            pose.orientation.z = vehicle_orientation.getZ();
            pose.orientation.w = vehicle_orientation.getW();

            v.object_polygon = ObjectToBoostPolygon<polygon_t>(pose, size);
            v.linear_velocity = veloctiy.linear;

            for(size_t i=1; i < tp.trajectory_points.size() - 1; i++){

                vehicle_vector = {tp.trajectory_points[i + 1].x - tp.trajectory_points[i].x , tp.trajectory_points[i+1].y - tp.trajectory_points[i].y};
                yaw = std::acos(vehicle_vector.dot(x_axis)/(vehicle_vector.norm() * x_axis.norm()));

                tf2::Quaternion orientation;
                orientation.setRPY(0, 0, yaw);

                geometry_msgs::Pose trajectory_pose;
                trajectory_pose.position.x = tp.trajectory_points[i].x;
                trajectory_pose.position.y = tp.trajectory_points[i].y;

                trajectory_pose.orientation.x = orientation.getX();
                trajectory_pose.orientation.y = orientation.getY();
                trajectory_pose.orientation.z = orientation.getZ();
                trajectory_pose.orientation.w = orientation.getW();

                std::tuple <__uint64_t,polygon_t> future_object(tp.trajectory_points[i].target_time.toNSec(),ObjectToBoostPolygon<polygon_t>(trajectory_pose, size));

                v.fp.push_back(future_object);
            }

            return v;
        };

        bool DetectCollision(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2, __uint64_t target_time) {            
            
            collision_detection::MovingObject ob_1_after = PredictObjectPosition(ob_1,target_time);

            collision_detection::MovingObject ob_2_after = PredictObjectPosition(ob_2,target_time);

            if(CheckPolygonIntersection(ob_1_after, ob_2_after)){
                return true;
            }
            
            return false;
        };

        bool CheckPolygonIntersection(collision_detection::MovingObject const &ob_1, collision_detection::MovingObject const &ob_2) {    

                std::deque<polygon_t> output;

                boost::geometry::intersection(ob_1.object_polygon, ob_2.object_polygon, output); 

                if(output.size() > 0){
                    return true;
                }

            return false;
        };

        collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject const &op, __uint64_t target_time){
            
            int union_polygon_size = 0;
            for (auto i : op.fp){
                if( std::get<0>(i) <= target_time) {
                    union_polygon_size = union_polygon_size + std::get<1>(i).outer().size();
                }
            }

            std::vector<point_t> unioin_future_polygon_points;
            unioin_future_polygon_points.reserve(union_polygon_size);

            for (auto i : op.fp){
                if( std::get<0>(i) <= target_time) {
                    unioin_future_polygon_points.insert( unioin_future_polygon_points.end(), std::get<1>(i).outer().begin(), std::get<1>(i).outer().end());
                }
            }

            polygon_t union_polygon;  
            boost::geometry::assign_points(union_polygon, unioin_future_polygon_points);

            polygon_t hull_polygon;
            boost::geometry::convex_hull(union_polygon, hull_polygon);

            collision_detection::MovingObject output_object = {hull_polygon, op.linear_velocity};
            
            return output_object;
        };

        template <class P>
        P ObjectToBoostPolygon(const geometry_msgs::Pose& pose, const geometry_msgs::Vector3& size) {

            tf2::Transform object_tf;
            tf2::fromMsg(pose, object_tf);

            double half_x_bound = size.x / 2;
            double half_y_bound = size.y / 2;

            // 4 corners of the object starting with upper left and moving in clockwise direction in pose frame
            tf2::Vector3 obj_p1(half_x_bound, half_y_bound, 0);
            tf2::Vector3 obj_p2(half_x_bound, -half_y_bound, 0);
            tf2::Vector3 obj_p3(-half_x_bound, -half_y_bound, 0);
            tf2::Vector3 obj_p4(-half_x_bound, half_y_bound, 0);

            tf2::Vector3 obj_p1_map = object_tf * obj_p1;
            tf2::Vector3 obj_p2_map = object_tf * obj_p2;
            tf2::Vector3 obj_p3_map = object_tf * obj_p3;
            tf2::Vector3 obj_p4_map = object_tf * obj_p4;

            point_t p1(obj_p1_map.getX(), obj_p1_map.getY());
            point_t p2(obj_p2_map.getX(), obj_p2_map.getY());
            point_t p3(obj_p3_map.getX(), obj_p3_map.getY());
            point_t p4(obj_p4_map.getX(), obj_p4_map.getY());

            P p;
            p.outer().push_back(p1);
            p.outer().push_back(p2);
            p.outer().push_back(p3);
            p.outer().push_back(p4);

            return p;
        }

    }
}