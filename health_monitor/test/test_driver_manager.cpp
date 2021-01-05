/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "driver_manager.h"
#include <gtest/gtest.h>

namespace health_monitor
{

//////////////////// Unit test for car part///////////////////////////////////////////////

    TEST(DriverManagerTest, testCarNormalDriverStatus)
    {
        // inordinary case where no critical drivers are specified
        DriverManager dm0;
        cav_msgs::DriverStatus msg0;
        msg0.controller = true;
        msg0.name = "controller";
        msg0.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg0_pointer(new cav_msgs::DriverStatus(msg0));
        dm0.update_driver_status(msg0_pointer, 1000);
        EXPECT_EQ("s_0", dm0.are_critical_drivers_operational_car(1500));
        
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);
        
        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_1", dm.are_critical_drivers_operational_car(1500));
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_0_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_0", dm.are_critical_drivers_operational_car(1500));
    }

        TEST(DriverManagerTest, testCarSsc_0_Lidar_1_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_car(1500));
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_0_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_0_g_1", dm.are_critical_drivers_operational_car(1500));
    }

    TEST(DriverManagerTest, testCarSsc_1_Lidar_1_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_1_l_1_g_0", dm.are_critical_drivers_operational_car(1500));
    }

        TEST(DriverManagerTest, testCarErrorDriverStatusTimeOut)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_car(2100));
    }
 //////////////////////////////////////////////////////////////////////////////////////////
 
    TEST(DriverManagerTest, testCarHandleSpinDriversReady)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(5, alert.type);
    }   

    TEST(DriverManagerTest, testCarHandleSpinCautionGpsNotWorkingLidarWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }   

    TEST(DriverManagerTest, testCarHandleSpinWarningLidarNotWorkingGpsWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(2, alert.type);
    }   

    TEST(DriverManagerTest, testCarHandleSpinFatalLidarNotWorkingGpsNotWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    } 

    TEST(DriverManagerTest, testCarHandleSpinFatalSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    } 

    TEST(DriverManagerTest, testCarHandleSpinFatalUnknownInside)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    } 

        TEST(DriverManagerTest, testCarHandleSpinNotReadyCase1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,1000,750);

        EXPECT_EQ(4, alert.type);
    } 

        TEST(DriverManagerTest, testCarHandleSpinNotReadyCase2)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=true;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,5000,750);

        EXPECT_EQ(4, alert.type);
    } 


    TEST(DriverManagerTest, testCarTruckHandleSpinFatalUnknown)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.gnss = true;
        msg3.name = "gps";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        bool truck=false;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(3, alert.type);
    } 


////////////////////////// Unit test for truck part////////////////////////////////////////
    
    TEST(DriverManagerTest, testTruckNormalDriverStatus)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_1", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_0_Lidar1_1_Lidar2_1_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_truck(1500));
    }


    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_0_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_0", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_0_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_0_g_1", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_1_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_0_Lidar2_1_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_0_l2_1_g_1", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_0_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_0_g_0", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_0_Gps_1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_0_g_1", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckSsc_1_Lidar1_1_Lidar2_1_Gps_0)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_1_l1_1_l2_1_g_0", dm.are_critical_drivers_operational_truck(1500));
    }

    TEST(DriverManagerTest, testTruckDriverStatusTimeout)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        EXPECT_EQ("s_0", dm.are_critical_drivers_operational_truck(2001));
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    

    TEST(DriverManagerTest, testHandleSpinDriverReady)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(5, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinCautionOneLidar1WorkingGpsWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinCautionLidar1WorkingGpsNotWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinCautionOneLidar2WorkingGpsWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::DEGRADED;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }

        TEST(DriverManagerTest, testHandleSpinCautionOneLidar1WorkingGpsNotWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }


        TEST(DriverManagerTest, testHandleSpinCautionOneLidar2WorkingGpsNotWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(1, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinWarningLidarNotWorkingGpsWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(2, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinFatalLidarNotWorkingGpsNotWorkingSscWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinFatalSscNotWorking)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    }

        TEST(DriverManagerTest, testHandleSpinFatalUnknownInsideTruck)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,150,750);

        EXPECT_EQ(6, alert.type);
    }


    TEST(DriverManagerTest, testHandleSpinNotReadyCase1)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,1000,750);

        EXPECT_EQ(4, alert.type);
    }

    TEST(DriverManagerTest, testHandleSpinNotReadyCase2)
    {
        std::vector<std::string> required_drivers{"controller"};
        std::vector<std::string> lidar_gps_drivers{"lidar1", "lidar2","gps"};

        DriverManager dm(required_drivers, 1000L,lidar_gps_drivers);

        cav_msgs::DriverStatus msg1;
        msg1.controller = true;
        msg1.name = "controller";
        msg1.status = cav_msgs::DriverStatus::OPERATIONAL;
        cav_msgs::DriverStatusConstPtr msg1_pointer(new cav_msgs::DriverStatus(msg1));
        dm.update_driver_status(msg1_pointer, 1000);
        
        cav_msgs::DriverStatus msg2;
        msg2.lidar = true;
        msg2.name = "lidar1";
        msg2.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg2_pointer(new cav_msgs::DriverStatus(msg2));
        dm.update_driver_status(msg2_pointer, 1000);

        cav_msgs::DriverStatus msg3;
        msg3.lidar = true;
        msg3.name = "lidar2";
        msg3.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg3_pointer(new cav_msgs::DriverStatus(msg3));
        dm.update_driver_status(msg3_pointer, 1000);

        cav_msgs::DriverStatus msg4;
        msg4.gnss = true;
        msg4.name = "gps";
        msg4.status = cav_msgs::DriverStatus::OFF;
        cav_msgs::DriverStatusConstPtr msg4_pointer(new cav_msgs::DriverStatus(msg4));
        dm.update_driver_status(msg4_pointer, 1000);

        bool truck=true;
        bool car=false;
        
        cav_msgs::SystemAlert alert;
        alert=dm.handleSpin(truck,car,1500,5000,750);

        EXPECT_EQ(4, alert.type);
    }

}