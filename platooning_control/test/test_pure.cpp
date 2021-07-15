#include "pure_pursuit.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>

// TEST(PurePursuitTest, test_filter)
// {
//     PlatooningControlPluginConfig config;
// 	config.lowpass_gain = 0.5;
    
//     platoon_control::PurePursuit pp;
//     pp.config_ = config;
//     double new_angle = pp.lowPassfilter(3.0);
//     EXPECT_EQ(1.5, new_angle);
// }

// TEST(PurePursuitTest, test_alpha)
// {
//     platoon_control::PurePursuit pp;
//     cav_msgs::TrajectoryPlanPoint tp;
//     tp.x = 0.0;
//     tp.y = 1.0;
//     geometry_msgs::Pose pose;
//     pose.position.x = 1;
//     pose.position.y = 0;
//     double out = pp.getAlpha(tp, pose);
//     EXPECT_EQ(out, -M_PI/2);
//     EXPECT_EQ(sin(out), -1);
// }

// TEST(PurePursuitTest, test1)
// {

//     cav_msgs::TrajectoryPlanPoint point;
//     point.x = 1.0;
//     point.y = 1.0;
//     point.target_time = ros::Time(1.0);
//     platoon_control::PurePursuit pp;
//     double res = pp.calculateSteer(point);
//     EXPECT_EQ(0, res);

//     cav_msgs::TrajectoryPlanPoint point2;
//     point2.x = 4.0;
//     point2.y = 4.0;
//     point2.target_time = ros::Time(2.0);
//     double res2 = pp.calculateSteer(point2);
//     EXPECT_NEAR(0.5, res2, 0.1);

// }


// TEST(PurePursuitTest, test2)
// {

//     cav_msgs::TrajectoryPlanPoint point;
//     point.x = 1.0;
//     point.y = 1.0;
//     point.target_time = ros::Time(1.0);
//     platoon_control::PurePursuit pp;
//     double res = pp.calculateSteer(point);
//     EXPECT_EQ(0, res);

//     cav_msgs::TrajectoryPlanPoint point2;
//     point2.x = -4.0;
//     point2.y = -4.0;
//     point2.target_time = ros::Time(1.0);
//     double res2 = pp.calculateSteer(point2);
//     EXPECT_NEAR(-0.5, res2, 0.1);

// }
