#include "pid_controller.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>


// TEST(PIDControllerTest, test1)
// {
//     platoon_control::PIDController pid;
//     double res = pid.calculate(40, 38);
//     EXPECT_EQ(-8, res);
// }


// TEST(PIDControllerTest, test2)
// {
//     platoon_control::PIDController pid;
//     double res = pid.calculate(20, 300);
//     EXPECT_EQ(100, res);
// }

// TEST(PIDControllerTest, test3)
// {
//     platoon_control::PIDController pid;
//     double res = pid.calculate(300, 20);
//     EXPECT_EQ(-100, res);
// }

// TEST(PIDControllerTest, test4)
// {
//     platoon_control::PIDController pid;
//     pid.reset();
//     double res = pid.calculate(200, 20);
//     double res2 = pid.calculate(500,25);
//     EXPECT_EQ(-100, res2);
//     double res3 = pid.calculate(25,500);
//     EXPECT_EQ(100, res3);
// }
