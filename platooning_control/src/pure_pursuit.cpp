
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

#include "pure_pursuit.hpp"



namespace platoon_control
{

	double PurePursuit::getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double x_diff = (tp.x-tp0.x);
		double y_diff = (tp.y-tp0.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
		return dist;
	}

	double PurePursuit::getVelocity(const cav_msgs::TrajectoryPlanPoint& tp, double delta_pos) const {
		ros::Duration delta_t = tp.target_time - tp0.target_time;
		double delta_t_second = fabs(delta_t.toSec());

		if(delta_t_second != 0) {
			return delta_pos / delta_t_second;
		}
		return 0.0;
	}

	double PurePursuit::getYaw() const{
		double yaw;
		yaw = atan2(current_pose_.orientation.x, current_pose_.orientation.z);
		return yaw;
	}

	double PurePursuit::getAlpha(double lookahead, std::vector<double> v1, std::vector<double> v2) const {
		
		double inner_prod = v1[0]*v2[0] + v1[1]*v2[1];
		double value = inner_prod/lookahead;
		if (value > 1) value = 1;
		else if (value < -1) value = -1;
		double alpha = acos(value);
        return alpha;
	}

	int PurePursuit::getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const{
		double corss_prod = v1[0]*v2[1] - v1[1]*v2[0];
        if (corss_prod >= 0.0){
			 return -1;
		}
        return 1;
	}

	double PurePursuit::calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp){
		// skip the first trajectory point
		if (tp0.x == 0 && tp0.y == 0){
			tp0 = tp;
			return 0.0;
		}
		double lookahead = getLookaheadDist(tp);
		double v = getVelocity(tp, lookahead);
		double yaw = getYaw();
		std::vector<double> v1 = {tp.x - current_pose_.position.x , tp.y - current_pose_.position.y};
		std::vector<double> v2 = {cos(yaw), sin(yaw)};
		double alpha = getAlpha(lookahead, v1, v2);
		int direction = getSteeringDirection(v1, v2);
		double steering = direction* atan((2 * wheelbase_ * sin(alpha))/(lookahead));// change (lookahead) to (Kdd_*v) if steering is bad
		tp0 = tp;
		if (std::isnan(steering)) return prev_steering;
		prev_steering = steering;
		return steering;
	}
}