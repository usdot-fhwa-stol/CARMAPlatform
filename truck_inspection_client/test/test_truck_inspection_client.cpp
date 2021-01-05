
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>

#include "truck_inspection_client.h"

#include "truck_inspection_client.h"

class TestInOutBoundSubscriber
{
    public:
        bool receivedMessage;
        cav_msgs::MobilityOperationConstPtr OutBoundMessage;

        TestInOutBoundSubscriber():receivedMessage(false) {}

        void OutBoundCallback(const cav_msgs::MobilityOperationConstPtr& newMessage)
        {
            ROS_INFO_STREAM("TestOutBoundSubscriber callback..");
            receivedMessage = true;
            OutBoundMessage = newMessage;
        }
        void InBoundCallback(const cav_msgs::MobilityRequestConstPtr& newMessage)
        {
            ROS_INFO_STREAM("TestInBoundSubscriber callback..");
            receivedMessage = true;
        }
};

TEST(TruckInspectionClientTest,TestMobilityOperationOutbound){
    ros::NodeHandle nh = ros::NodeHandle();
    TestInOutBoundSubscriber subscriber_InBound;
    TestInOutBoundSubscriber subscriber_OutBound;

    // Subscribers
    ros::Subscriber mobility_request_inbound_sub = nh.subscribe("mobility_request_inbound", 5,&TestInOutBoundSubscriber::InBoundCallback, &subscriber_InBound);    
    ros::Subscriber mobility_operation_outbound_sub = nh.subscribe("mobility_operation_outbound",0,&TestInOutBoundSubscriber::OutBoundCallback,&subscriber_OutBound); 
    std::this_thread::sleep_for(std::chrono::milliseconds(35000));
    
    // Spin so that publication can get to subscription
    ros::spinOnce(); 

    //ASSERTION
    EXPECT_EQ(0, mobility_operation_outbound_sub.getNumPublishers()); 
    EXPECT_FALSE(subscriber_OutBound.receivedMessage);

    //mobility_request_inbound publish data 
    if(mobility_request_inbound_sub.getNumPublishers() > 0 && subscriber_InBound.receivedMessage 
        && subscriber_OutBound.OutBoundMessage && subscriber_OutBound.OutBoundMessage->strategy_params.c_str()!=NULL){
        EXPECT_EQ("vin_number:,license_plate:DOT-10003,carrier_name:FMCSA Tech Division,carrier_id:DOT 12,weight:16639,ads_software_version:System Version Unknown,date_of_last_state_inspection:2020.01.01,date_of_last_ads_calibration:2020.02.01,pre_trip_ads_health_check:Green,ads_health_status:6,ads_auto_status:Not Engaged,iss_score:25,permit_required:0",subscriber_OutBound.OutBoundMessage->strategy_params);
    }
    else if(subscriber_OutBound.OutBoundMessage && subscriber_OutBound.OutBoundMessage->strategy_params.c_str()!=NULL)
    {
        EXPECT_NE("vin_number:,license_plate:DOT-10003,state_short_name:VA",subscriber_OutBound.OutBoundMessage->strategy_params);
    }
}

int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "truck_inspection_client_node_test");
    std::thread spinner([] {while (ros::ok()) ros::spin();});
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}