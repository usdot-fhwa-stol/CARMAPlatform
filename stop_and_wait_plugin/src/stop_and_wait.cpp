/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include "stopandwait.h"
#include <vector>

#include <cav_msgs/Trajectory.h>
#include <cav_msgs/StopAndWaitManeuver.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>



using oss = std::ostringstream;

namespace stopandwait_plugin
{
    StopandWait::StopandWait(carma_wm::WorldModelConstPtr wm, PublishPluginDiscoveryCB plugin_discovery_publisher)
    : wm_(wm) , plugin_discovery_publisher_(plugin_discovery_publisher)
    {
        plugin_discovery_msg_.name = "StopandWaitPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
    }

    bool StopandWait::onSpin()
    {
        plugin_discovery_publisher_(plugin_discovery_msg_);
        return true;
    }

    bool StopandWait::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        //ros::WallTime start_time = ros::WallTime::now(); - for checking planning time
        
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global,req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points(req.maneuver_plan.maneuvers, current_downtrack, wm_);

        auto downsampled_points = 
            carma_utils::containers::downsample_vector(points_and_target_speeds,8);

        //Trajectory plan
        cav_msgs::TrajectoryPlan  trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points,req.vehicle_state);
        trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;

        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::STOP_AND_WAIT);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        // ros::WallTime end_time = ros::WallTime::now(); // Planning complete
        // ros::WallDuration duration = end_time - start_time;
        // ROS_DEBUG_STREAM("ExecutionTime: " << duration.toSec());

        return true;
    }

    std::vector<PointSpeedPair> StopandWait::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm)
    {
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        for(const auto& maneuver : maneuvers)
        {
            if(maneuver.type != cav_msgs::Maneuver::STOP_AND_WAIT)
            {
                throw std::invalid_argument ("Stop and Wait Maneuver Plugin doesn't support this maneuver type");
            }
            cav_msgs::StopAndWaitManeuver stop_and_wait_maneuver= maneuver.stop_and_wait_maneuver;

            double starting_downtrack = stop_and_wait_maneuver.start_dist;  //starting downtrack recorded in message
            if(first)   //check for first maneuver in vector 
            {
                if(starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
                first = false;

            }
           
            ROS_DEBUG_STREAM("Used downtrack: " << starting_downtrack);
            double ending_downtrack = stop_and_wait_maneuver.end_dist;
            
            //------------------------------------------
            //Create speed profile
            double maneuver_time = ros::Duration(stop_and_wait_maneuver.end_time - stop_and_wait_maneuver.start_time).toSec();
            //double travel_dist =  stop_and_wait_maneuver.end_dist - starting_downtrack;
            //double acceleration = stop_and_wait_maneuver.start_speed/maneuver_time;
            double jerk_req, jerk;
            jerk_req = (2*stop_and_wait_maneuver.start_speed)/pow(maneuver_time,2);
            if(jerk_req > max_jerk_limit_)
            {
                //unsafe to stop at the required jerk - reset to max_jerk and go beyond the maneuver end_dist
                jerk = max_jerk_limit_;  
                maneuver_time = pow(2*stop_and_wait_maneuver.start_speed, 0.5);
                double travel_dist_new = stop_and_wait_maneuver.start_speed * maneuver_time - (0.167 * jerk * pow(maneuver_time,3));
                ending_downtrack = travel_dist_new + starting_downtrack;
            }
            else jerk = jerk_req;

            //----------------------------------------------

            //get all the lanelets in between starting and ending downtrack on shortest path
            auto lanelets = wm_->getLaneletsBetween(starting_downtrack, ending_downtrack, true);
            
            //record all the lanelets to be added to path
            std::vector<lanelet::ConstLanelet> lanelets_to_add;
            for (auto l : lanelets)
            {
                ROS_DEBUG_STREAM("Lanelet ID: " << l.id());
                if(visited_lanelets.find(l.id()) == visited_lanelets.end())
                {
                    lanelets_to_add.push_back(l);
                    visited_lanelets.insert(l.id());
                }
            }

            lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);
            
            int points_count = route_geometry.size();
            double delta_time = maneuver_time/(points_count-1);

            first = true;
            double curr_time = 0.0;

            for(auto p : route_geometry)
            {
                if (first && points_and_target_speeds.size() != 0)
                {
                    first = false;

                    continue; //Skip the first point to avoid duplicates from previous maneuver
                }
                PointSpeedPair pair;
                pair.point = p;
                pair.speed = stop_and_wait_maneuver.start_speed - (0.5 * jerk_req * pow(curr_time,2));
                if(p == route_geometry.back()) pair.speed = 0.0;    //force speed to 0 at last point
                curr_time +=delta_time;

                points_and_target_speeds.push_back(pair);
            }
 
            if(maneuver_time < minimal_trajectory_duration_)
            {
                while(curr_time < minimal_trajectory_duration_)
                {
                    PointSpeedPair pair;
                    pair.point = points_and_target_speeds.back().point;
                    pair.speed = 0.0;
                    curr_time += delta_time;
                }
            }
        }
        
        return points_and_target_speeds;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> StopandWait::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {        
        int nearest_pt_index = getNearestPointIndex(points,state);
        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position

        //Get yaw - geometrically
        std::vector<float> yaw_values;
        for(size_t i=0 ;i < points.size()-1 ;i++)
        {
            float yaw = atan((points[i+1].point.y() - points[i].point.y())/ (points[i+1].point.x() - points[i].point.x()));
            yaw_values.push_back(yaw);
        }
        yaw_values.push_back(0.0); //No rotation from last point

        //get target time from speed
        std::vector<double> target_times;
        std::vector<lanelet::BasicPoint2d> trajectory_locations;
        std::vector<double> trajectory_speeds;
        //split point speed pair
        splitPointSpeedPairs(points,&trajectory_locations,&trajectory_speeds);
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(trajectory_locations);

        trajectory_utils::conversions::speed_to_time(downtracks, trajectory_speeds, &target_times);

        std::vector <cav_msgs::TrajectoryPlanPoint> traj;
        ros::Time start_time = ros::Time::now();
        for (size_t i=0; i < points.size(); i++)
        {

            cav_msgs::TrajectoryPlanPoint traj_point;
            traj_point.x = points[i].point.x();
            traj_point.y = points[i].point.y();
            traj_point.yaw = yaw_values[i];
            traj_point.target_time = start_time + ros::Duration(target_times[i]);
            traj.push_back(traj_point);
        }
        
        return traj;
    }

    int StopandWait::getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            ROS_DEBUG_STREAM("distance: " << distance);
            ROS_DEBUG_STREAM("p: " << p.point.x() << ", " << p.point.y());
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;

    }

    void StopandWait::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                        std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

}