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

#include <gtest/gtest.h>
#include "mock_lightbar_driver/MockLightBarDriver.hpp"
#include <ros/ros.h>
#include <ros/service_server.h>
#include "carma_utils/CARMANodeHandle.h"
using namespace mock_drivers;


    TEST(GetLightsBasicTest,testGetLights){
        ros::CARMANodeHandle n;
        ros::ServiceClient client = n.serviceClient<cav_srvs::GetLights>("lightbar/get_lights");
        cav_srvs::GetLights light;
        ROS_INFO_STREAM(" " << client.getService());
        if (client.call(light.request,light.response)){
             ROS_INFO("called");
              ASSERT_TRUE(light.response.status.green_flash == 0 && light.response.status.flash == 0 && light.response.status.left_arrow == 0
                && light.response.status.right_arrow == 0 && light.response.status.sides_solid == 0 && light.response.status.green_solid == 0
                && light.response.status.yellow_solid == 0 && light.response.status.takedown == 0);
         } else {
             ROS_INFO("FAILED");
         }
    }

    TEST(SetLightsTest,testSetLights){
         ros::CARMANodeHandle n;
        ros::ServiceClient client = n.serviceClient<cav_srvs::SetLights>("lightbar/set_lights");
        ros::ServiceClient client2 = n.serviceClient<cav_srvs::GetLights>("lightbar/get_lights");
        cav_srvs::GetLights light1;
        cav_srvs::SetLights light;
        light.request.set_state.green_flash = cav_msgs::LightBarStatus::ON;
        light.request.set_state.flash = cav_msgs::LightBarStatus::ON;
        light.request.set_state.left_arrow = cav_msgs::LightBarStatus::ON;
        light.request.set_state.right_arrow = cav_msgs::LightBarStatus::ON;
        light.request.set_state.sides_solid = cav_msgs::LightBarStatus::OFF;
        light.request.set_state.green_solid = cav_msgs::LightBarStatus::OFF;
        light.request.set_state.yellow_solid = cav_msgs::LightBarStatus::OFF;
        light.request.set_state.takedown = cav_msgs::LightBarStatus::OFF;
        if (client.call(light)){
            if (client2.call(light1)){
                 ASSERT_TRUE(light1.response.status.green_flash == 1 && light1.response.status.flash == 1 && light1.response.status.left_arrow == 1
                && light1.response.status.right_arrow == 1 && light1.response.status.sides_solid == 0 && light1.response.status.green_solid == 0
                && light1.response.status.yellow_solid == 0 && light1.response.status.takedown == 0);
            } 
        } 
    }

    TEST(GetDriverAPITest,testGetDriverAPI){
        ros::CARMANodeHandle n;
        ros::ServiceClient client = n.serviceClient<cav_srvs::GetDriverApi>("get_driver_api");
        cav_srvs::GetDriverApi api;
        if (client.call(api)){
            ASSERT_TRUE(api.response.api_list.size() == 3 && api.response.api_list[0] == "lightbar/light_bar_status" 
            && api.response.api_list[1] == "lightbar/get_lights" && api.response.api_list[2] == "lightbar/set_lights");
        }
    }

    TEST(GetDriverStatusTest,testGetDriverStatus){
        ros::CARMANodeHandle n;
        ros::ServiceClient client = n.serviceClient<cav_srvs::GetDriverStatus>("get_status");
        cav_srvs::GetDriverStatus status;
        if (client.call(status)){
            ASSERT_TRUE(status.response.status.lightbar == true && status.response.status.camera == false && status.response.status.can == false
            && status.response.status.comms == false && status.response.status.controller == false && status.response.status.gnss == false
            && status.response.status.imu == false && status.response.status.lidar == false && status.response.status.radar == false
            && status.response.status.roadway_sensor == false && status.response.status.status == 1 && status.response.status.name == "MockLightBarDriver");
        }
    }
    void discovery_cb(const cav_msgs::DriverStatus& status){
        ASSERT_TRUE(status.lightbar == true && status.camera == false && status.can == false
            && status.comms == false && status.controller == false && status.gnss == false
            && status.imu == false && status.lidar == false && status.radar == false
            && status.roadway_sensor == false && status.status == 1 && status.name == "MockLightBarDriver");

    }

    TEST(DriverStatusPubTest,testDriverStatusPublish){
        ros::CARMANodeHandle n;
        cav_msgs::DriverStatus status;
        ros::Subscriber sub = n.subscribe("driver_discovery",10,&discovery_cb);
        ros::spinOnce();
    }

    void lightbar_cb(const cav_msgs::LightBarStatus& status){
        ASSERT_TRUE(status.green_flash == 1 && status.flash == 1 && status.left_arrow == 1
                && status.right_arrow == 1 && status.sides_solid == 0 && status.green_solid == 0
                && status.yellow_solid == 0 && status.takedown == 0);
    }
    TEST(LightBarStatusPubTest, testLightBarStatusPublish){
        ros::CARMANodeHandle n;
        cav_msgs::LightBarStatus status;
        ros::Subscriber sub = n.subscribe("lightbar/light_bar_status",10,&lightbar_cb);
        ros::spinOnce();
    }
  

    int main(int argc, char** argv){
        testing::InitGoogleTest(&argc,argv);
        ros::init(argc,argv, "test_mock_light_bar_driver");
        auto result = RUN_ALL_TESTS();
        return result;

    }
