#pragma once

#include <ros/ros.h>
#include <math.h> 
#include <iostream>
// #include <Eigen/Core>
// #include <Eigen/LU>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cav_msgs/TrajectoryPlan.h>







namespace platoon_control
{


    class PurePursuit
    {


    public:
    	PurePursuit();

    	double calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp);

		// geometry pose
		geometry_msgs::Pose current_pose_;

    private:

		// calculate the lookahead distance from next trajectory point
		double getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const;

		// calculate yaw angle of the vehicle
		double getYaw() const;

		// calculate alpha angle
		double getAlpha(double lookahead, std::vector<double> v1, std::vector<double> v2) const;

		// calculate steering direction
		int getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const;

		// calculate command velocity from trajecoty point
		double getVelocity(const cav_msgs::TrajectoryPlanPoint& tp, double delta_pos) const;
		
		// vehicle wheel base
		double wheelbase_ = 2.7;

		// coefficient for smooth steering
		double Kdd_ = 4.5;

		double prev_steering = 0.0;

		
		// previous trajectory point
		cav_msgs::TrajectoryPlanPoint tp0;

		// helper function (if needed)
		// inline double deg2rad(double deg) const
		// {
		// 	return deg * M_PI / 180;
		// }  // convert degree to radian


    };
}