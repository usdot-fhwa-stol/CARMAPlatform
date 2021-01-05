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



#include <ros/ros.h>
#include <ros/node_handle.h>
#include <iostream>
#include <ros/service_client.h>
#include <ros/service_server.h>
#include "mock_lightbar_driver/MockLightBarDriver.hpp"
#include <cav_srvs/GetLights.h>
#include <cav_srvs/GetLightsRequest.h>
#include <cav_srvs/GetLightsResponse.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetLightsRequest.h>
#include <cav_srvs/SetLightsResponse.h>
#include <cav_msgs/LightBarStatus.h>
using namespace std;

namespace mock_drivers{
            MockLightBarDriver::MockLightBarDriver(ros::CARMANodeHandle node)  {   
                //lightbar topics and services
                lbStatusTopic = "lightbar/light_bar_status";
                lbGetLightService = "lightbar/get_lights";
                lbSetLightService = "lightbar/set_lights";
                //lightbar states
                greenFlash = false;
                yellowFlash = false;
                leftArrow = false;
                rightArrow = false;
                sidesSolid = false;
                greenSolid = false;
                yellowDim = false;
                takedown = false;      
                //topics published
                lbPub = node.advertise<cav_msgs::LightBarStatus>(lbStatusTopic, 100);
                discoveryPub = node.advertise<cav_msgs::DriverStatus>("driver_discovery",100);  
                //service server
                getApiService = node.advertiseService("mock_lightbar/get_driver_api",&MockLightBarDriver::getApiService_cb, this);
                bindService = node.advertiseService("mock_lightbar/bind",&MockLightBarDriver::bind_cb, this);
                getDriverStatusService = node.advertiseService("mock_lightbar/get_status",&MockLightBarDriver::getDriverStatus_cb,this);
                setLightsService = node.advertiseService(lbSetLightService, &MockLightBarDriver::setLightService_cb, this);
                getLightsService = node.advertiseService(lbGetLightService, &MockLightBarDriver::getLightService_cb, this);
                //publish driver status
                driverStatus.status = cav_msgs::DriverStatus::OPERATIONAL;
                publishDriverStatus();
            }

            //get driver status call back
            bool MockLightBarDriver::getDriverStatus_cb(cav_srvs::GetDriverStatusRequest& req, cav_srvs::GetDriverStatusResponse& resp){
                resp.status = getDriverStatus();
                return true;
            }

            //bind service call back
            bool MockLightBarDriver::bind_cb(cav_srvs::BindRequest& req, cav_srvs::BindResponse& resp){
                ROS_INFO_STREAM("bind received");
                return true;
            }

            //get api service call back
            bool MockLightBarDriver::getApiService_cb(cav_srvs::GetDriverApiRequest& req, cav_srvs::GetDriverApiResponse& resp){
                resp.api_list = getDriverApi();
                return true;
            }

            //set light service call back
            bool MockLightBarDriver::setLightService_cb(cav_srvs::SetLightsRequest& req, cav_srvs::SetLightsResponse& resp){
                cav_msgs::LightBarStatus lightstatus = req.set_state;
                greenFlash = lightstatus.green_flash == 1;
                yellowFlash = lightstatus.flash == 1;
                leftArrow = lightstatus.left_arrow == 1;
                rightArrow = lightstatus.right_arrow== 1;
                sidesSolid = lightstatus.sides_solid == 1;
                greenSolid = lightstatus.green_solid == 1;
                yellowDim = lightstatus.yellow_solid == 1;
                takedown = lightstatus.takedown == 1;
                return true;
            }

            //get light service call back
            bool MockLightBarDriver::getLightService_cb(cav_srvs::GetLightsRequest& req, cav_srvs::GetLightsResponse& resp){
                resp.status = getLightBarStatus();
                return true;
            }

            /**
            * Helper function to build the lightbar status message
            * @return The lightbar status message
            */
            cav_msgs::LightBarStatus MockLightBarDriver::getLightBarStatus(){
                cav_msgs::LightBarStatus status;
                status.green_solid = greenSolid? 1:0;
                status.yellow_solid = yellowDim? 1:0;
                status.right_arrow = rightArrow? 1:0;
                status.left_arrow = leftArrow? 1:0;
                status.sides_solid = sidesSolid? 1:0;
                status.flash = yellowFlash? 1:0;
                status.green_flash = greenFlash? 1:0;
                status.takedown = takedown? 1:0;
                return status;

            }

            cav_msgs::DriverStatus MockLightBarDriver::getDriverStatus(){
                cav_msgs::DriverStatus status;
                status.name = getNodeName();
                status.status = driverStatus.status;
                status.can = false;
                status.radar = false;
                status.gnss = false;
                status.imu = false;
                status.lidar = false;
                status.roadway_sensor = false;
                status.comms = false;
                status.controller = false;
                status.camera = false;
                status.lightbar = true;
                return status;

            }

            vector<string> MockLightBarDriver::getDriverApi()  {
                vector<string> api;
                api.push_back(lbStatusTopic);
                api.push_back(lbGetLightService);
                api.push_back(lbSetLightService);
                return api;

            } 
        
            void MockLightBarDriver::publishData()  {
                cav_msgs::LightBarStatus status = getLightBarStatus();
                lbPub.publish(status);
            }

            void MockLightBarDriver::publishDriverStatus(){
                discoveryPub.publish(getDriverStatus());
            }

            string MockLightBarDriver::getNodeName(){
                return "MockLightBarDriver";
            }

            vector<string> MockLightBarDriver::getDriverTypesList() {
                vector<string> list;
                list.push_back("lightbar");
                return list;
            }


}