#include "ros/ros.h"
#include <cav_srvs/PlanTrajectory.h>

bool callback(cav_srvs::PlanTrajectory::Request  &req,
         cav_srvs::PlanTrajectory::Response &res)
{
    if (req.initial_trajectory_plan.trajectory_id == "YieldReq"){
        res.trajectory_plan.trajectory_id = "YieldResp";
    } 
    ROS_ERROR("Yield callback");
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "helper");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plugins/YieldPlugin/plan_trajectory", callback);
  ros::spin();
  ros::Duration(5).sleep();

  return 0;
}