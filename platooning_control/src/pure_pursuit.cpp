#include "pure_pursuit.hpp"
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>



namespace platoon_control
{
	PurePursuit::PurePursuit(){}

	double PurePursuit::getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double x_diff = (tp.x - current_pose_.position.x);
		double y_diff = (tp.y - current_pose_.position.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
		ROS_DEBUG_STREAM("calculated lookahead: " << dist);
		return dist;
	}


	double PurePursuit::getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double yaw = atan2(tp.y - current_pose_.position.y, tp.x - current_pose_.position.x);
		return yaw;
	}

	int PurePursuit::getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const{
		double corss_prod = v1[0]*v2[1] - v1[1]*v2[0];
        if (corss_prod >= 0.0){
			 return -1;
		}
        return 1;
	}

	double PurePursuit::getSteeringAngle()
	{
		return steering_angle_;
	}
		
	double PurePursuit::getAngularVelocity()
	{
		return angular_velocity_;
	}

	double PurePursuit::calculateKappa(const cav_msgs::TrajectoryPlanPoint& tp)
	{
		double lookahead = getLookaheadDist(tp);
		ROS_DEBUG_STREAM("used lookahead: " << lookahead);
		double alpha = getAlphaSin(tp, current_pose_);
		ROS_DEBUG_STREAM("calculated alpha: " << alpha);
		double kappa = 2 * (alpha)/(lookahead);
		ROS_DEBUG_STREAM("calculated kappa: " << kappa);
		return kappa;
	}

	void PurePursuit::calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp)
	{

		double kappa = calculateKappa(tp);
		
		double steering = atan(config_.wheelbase * kappa);
		ROS_DEBUG_STREAM("calculated steering angle: " << steering);
		double filtered_steering = lowPassfilter(config_.lowpass_gain, prev_steering, steering);
		ROS_DEBUG_STREAM("filtered steering: " << filtered_steering);
		if (std::isnan(filtered_steering)) filtered_steering = prev_steering;
		prev_steering = filtered_steering;
		steering_angle_ = filtered_steering;
		
		double ang_vel = velocity_ * kappa;
		ROS_DEBUG_STREAM("calculated angular velocity: " << ang_vel);
		double filtered_ang_vel = lowPassfilter(config_.lowpass_gain, prev_ang_vel, ang_vel);
		ROS_DEBUG_STREAM("filtered angular velocity: " << filtered_ang_vel);
		prev_ang_vel = filtered_ang_vel;
		if (std::isnan(filtered_ang_vel)) filtered_ang_vel = prev_ang_vel;
		angular_velocity_ = filtered_ang_vel;
	}

	double PurePursuit::getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose)
	{
		tf::Transform inverse;
		tf::poseMsgToTF(current_pose, inverse);
		tf::Transform transform = inverse.inverse();

		geometry_msgs::Point point_msg;
		point_msg.x = tp.x;
		point_msg.y = tp.y;
		point_msg.z = current_pose.position.z;
		tf::Point p;
		pointMsgToTF(point_msg, p);
		tf::Point tf_p = transform * p;
		geometry_msgs::Point tf_point_msg;
		pointTFToMsg(tf_p, tf_point_msg);
		ROS_DEBUG_STREAM("relative latitude: " << tf_point_msg.y);
		ROS_DEBUG_STREAM("relative longitude: " << tf_point_msg.x);
		ROS_DEBUG_STREAM("relative z: " << tf_point_msg.z);
		double vec_mag = std::sqrt(tf_point_msg.y*tf_point_msg.y + tf_point_msg.x*tf_point_msg.x + tf_point_msg.z*tf_point_msg.z);
		ROS_DEBUG_STREAM("relative vector mag: " << vec_mag);
		double sin_alpha = tf_point_msg.y/vec_mag;
		ROS_DEBUG_STREAM("alpha sin from transform: " << sin_alpha);

		double angle_tp_map = atan2(tp.y, tp.x); // angle of vector to tp point in map frame
		ROS_DEBUG_STREAM("angle_tp_map: " << angle_tp_map);
		tf::Quaternion quat;
		tf::quaternionMsgToTF(current_pose.orientation, quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		ROS_DEBUG_STREAM("yaw: " << yaw);
		double alpha = angle_tp_map - yaw;
		double sin_alpha2 = sin(alpha);
		ROS_DEBUG_STREAM("alpha from orientation: " << alpha);
		ROS_DEBUG_STREAM("alpha sin from orientation: " << sin_alpha2);

		return sin_alpha;
	}

	double PurePursuit::lowPassfilter(double gain, double prev_value, double value)
	{	
		// angle = config_.lowpass_gain * angle + (1 - config_.lowpass_gain) * prev_steering;
		value = prev_value + gain*(value - prev_value);
    	return value;
	}
}