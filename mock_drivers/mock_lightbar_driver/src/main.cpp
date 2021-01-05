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
#include "carma_utils/CARMANodeHandle.h"
#include "mock_lightbar_driver/MockLightBarDriver.hpp"
using namespace std;
using namespace mock_drivers;



int main(int argc, char** argv){
    

    ros::init(argc,argv,"MockLightBarDriver");
    ros::CARMANodeHandle n;
    MockLightBarDriver f (n);
    ROS_INFO("running"); 
    ros::Rate loop_rate(10);
    while (ros::ok()){
        f.publishData();
        f.publishDriverStatus();
        ros::spinOnce();
        loop_rate.sleep();
    }
}