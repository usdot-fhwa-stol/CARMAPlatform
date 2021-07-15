#include "platoon_control.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


// TEST(PlatoonControlPluginTest, test1)
// {

//     platoon_control::PlatoonControlPlugin pc;
    
//     ros::Time::init();

//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = -1.0;
//     pc.pose_msg_.reset(new geometry_msgs::PoseStamped(pose));

//     cav_msgs::TrajectoryPlanPoint point;
//     point.x = 1.0;
//     point.y = 2.0;
//     point.target_time = ros::Time(0, 10);
//     geometry_msgs::TwistStamped res_twist;
//     res_twist = pc.composeTwist(point);
//     EXPECT_NEAR(0.0,res_twist.twist.linear.x, 0.1);

// }

// TEST(PlatoonControlPluginTest, test2)
// {
//     cav_msgs::TrajectoryPlan tp;
//     cav_msgs::TrajectoryPlanPoint point1;
//     point1.x = 1.0;
//     point1.y = 1.0;
    

//     cav_msgs::TrajectoryPlanPoint point2;
//     point2.x = 10.0;
//     point2.y = 10.0;

//     cav_msgs::TrajectoryPlanPoint point3;
//     point3.x = 20.0;
//     point3.y = 20.0;

//     tp.trajectory_points = {point1, point2, point3};



//     platoon_control::PlatoonControlPlugin pc;
//     pc.current_speed_ = 5;
//     cav_msgs::TrajectoryPlanPoint out = pc.getLookaheadTrajectoryPoint(tp);
//     EXPECT_EQ(out.x, 10.0);
// }



