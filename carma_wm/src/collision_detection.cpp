#include "carma_wm/collision_detection.h"

namespace carma_wm {

    namespace collision_detection {

        std::vector<cav_msgs::RoadwayObstacle> WorldCollisionDetection(cav_msgs::RoadwayObstacleList& rwol, cav_msgs::TrajectoryPlan& tp, geometry_msgs::Vector3& size, geometry_msgs::Twist& veloctiy, unsigned int target_time){

            std::vector<cav_msgs::RoadwayObstacle> rwo_collison;

            collision_detection::MovingObject vehicle_object = ConvertVehicleToMovingObject(tp, size, veloctiy);

            for (size_t i = 0; i < rwol.roadway_obstacles.size(); i++) {

                collision_detection::MovingObject rwo = ConvertRoadwayObstacleToMovingObject(rwol.roadway_obstacles[i]);

                bool collision = DetectCollision(vehicle_object, rwo, target_time);

                if(collision) {
                    rwo_collison.push_back(rwol.roadway_obstacles[i]);
                }
            }

            return rwo_collison;
        };

        collision_detection::MovingObject ConvertRoadwayObstacleToMovingObject(cav_msgs::RoadwayObstacle& rwo){

            collision_detection::MovingObject ro;
            
            ro.object_polygon = ObjectToBoostPolygon<polygon_t>(rwo.object.pose.pose, rwo.object.size);

            for (auto i : rwo.object.predictions){
                ro.future_polygons.push_back(ObjectToBoostPolygon<polygon_t>(i.predicted_position, rwo.object.size));
            }

            ro.linear_velocity = rwo.object.velocity.twist.linear;

            return ro;
        };

        collision_detection::MovingObject ConvertVehicleToMovingObject(cav_msgs::TrajectoryPlan& tp, geometry_msgs::Vector3& size, geometry_msgs::Twist& veloctiy){
            
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

                geometry_msgs::Pose pose;
                pose.position.x = tp.trajectory_points[i].x;
                pose.position.y = tp.trajectory_points[i].y;

                pose.orientation.x = orientation.getX();
                pose.orientation.y = orientation.getY();
                pose.orientation.z = orientation.getZ();
                pose.orientation.w = orientation.getW();

                v.future_polygons.push_back(ObjectToBoostPolygon<polygon_t>(pose, size));
            }

            return v;
        };

        bool DetectCollision(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2, unsigned int target_time) {            

            collision_detection::MovingObject ob_1_after = PredictObjectPosition(ob_1,target_time);
            collision_detection::MovingObject ob_2_after = PredictObjectPosition(ob_2,target_time);

            if(CheckPolygonIntersection(ob_1_after, ob_2_after)){
                return true;
            }
            
            return false;
        };

        bool CheckPolygonIntersection(collision_detection::MovingObject ob_1, collision_detection::MovingObject ob_2) {    

                std::deque<polygon_t> output;

                boost::geometry::intersection(ob_1.object_polygon, ob_2.object_polygon, output); 

                if(output.size() > 0){
                    return true;
                }

            return false;
        };

        collision_detection::MovingObject PredictObjectPosition(collision_detection::MovingObject op, unsigned int target_time){

            int size = 0;
            for(size_t i = 0; i< target_time; i++){
                size = size + op.future_polygons[i].outer().size();
            }

            std::vector<point_t> unioin_polygon_points;

            unioin_polygon_points.reserve(size); 

            for(size_t i = 0; i< target_time; i++){
                unioin_polygon_points.insert( unioin_polygon_points.end(), op.future_polygons[i].outer().begin(), op.future_polygons[i].outer().end());
            }

            polygon_t union_polygon;  
            boost::geometry::assign_points(union_polygon, unioin_polygon_points);

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