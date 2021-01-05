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


#include "truck_inspection_client.h"

namespace truck_inspection_client
{

    void TruckInspectionClient::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->getParam("/vin_number", vin_number_); //get vin from global params in UniqueVehicleParams.ymal
        pnh_->getParam("license_plate", license_plate_);
        pnh_->getParam("state_short_name", state_short_name_);
        pnh_->getParam("carrier_name", carrier_name_);
        pnh_->getParam("carrier_id", carrier_id_);
        pnh_->getParam("weight", weight_);
        pnh_->getParam("date_of_last_state_inspection", date_of_last_state_inspection_);
        pnh_->getParam("date_of_last_ads_calibration", date_of_last_ads_calibration_);
        pnh_->getParam("iss_score", iss_score_);
        pnh_->getParam("permit_required", permit_required_);
        pnh_->getParam("pre_trip_ads_health_check", pre_trip_ads_health_check_);
        mo_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 5);
        request_sub_ = nh_->subscribe("incoming_mobility_request", 1, &TruckInspectionClient::requestCallback, this);
        ads_state_sub_ = nh_->subscribe("guidance/state", 1, &TruckInspectionClient::guidanceStatesCallback, this);
        ads_system_alert_sub_ = nh_->subscribe("system_alert", 1, &TruckInspectionClient::systemAlertsCallback, this);
        version_sub_ = nh_->subscribe("carma_system_version", 1, &TruckInspectionClient::versionCallback, this);
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &TruckInspectionClient::bsmCallback, this);
        this->ads_engaged_ = false;
        this->ads_system_alert_type_ = std::to_string(cav_msgs::SystemAlert::NOT_READY);
        this->ads_software_version_ = "System Version Unknown";
        //Checking: vin number does not exist yet in global parameter server
        while((!pnh_->getParam("/vin_number", vin_number_)) && vin_retrive_count < MAX_RETRIEVE_VIN_COUNT)
        {
            //sleep for 0.1 second
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            vin_retrive_count++;

            if(pnh_->getParam("/vin_number", vin_number_))
            {
                ROS_INFO("retrieved vin_number1:  %s",vin_number_.c_str());
                break; 
            }
            ROS_DEBUG("retrieving vin_number:  %s",vin_number_.c_str());
        }
        //if exceed maximum count, publish warning messages to system_alert
        if(vin_retrive_count >= MAX_RETRIEVE_VIN_COUNT)
        {
            cav_msgs::SystemAlert alert_msg;
            alert_msg.type = cav_msgs::SystemAlert::WARNING;
            alert_msg.description = ros::this_node::getName() + ": vin_number does not exist in global parameter server.";                
            ROS_ERROR_STREAM(alert_msg.description);
            nh_->publishSystemAlert(alert_msg);
        }

        // set vin publisher
        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            cav_msgs::MobilityOperation msg_out;
            msg_out.strategy = this->INSPECTION_STRATEGY;
            msg_out.strategy_params = "vin_number:" + vin_number_ + ",license_plate:" + license_plate_ +",state_short_name:"+ state_short_name_;
            mo_pub_.publish(msg_out);
            return true;
        });
        ROS_INFO_STREAM("Truck inspection plugin is initialized...");
    }

    void TruckInspectionClient::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void TruckInspectionClient::bsmCallback(const cav_msgs::BSMConstPtr& msg)
    {
        auto id_vector = msg->core_data.id;
        bsm_id_ = "";
        for(auto c : id_vector) bsm_id_ += std::to_string(c);

    }

    void TruckInspectionClient::guidanceStatesCallback(const cav_msgs::GuidanceStateConstPtr& msg)
    {
        this->ads_engaged_ = (msg->state == cav_msgs::GuidanceState::ENGAGED);
    }
    void TruckInspectionClient::systemAlertsCallback(const cav_msgs::SystemAlertConstPtr& msg){
        this->ads_system_alert_type_ = std::to_string(msg->type);
    }
    void TruckInspectionClient::requestCallback(const cav_msgs::MobilityRequestConstPtr& msg)
    {
        if(msg->strategy == this->INSPECTION_STRATEGY) {           
            cav_msgs::MobilityOperation mo_msg;
            mo_msg.header.sender_bsm_id = bsm_id_;
            mo_msg.strategy = this->INSPECTION_STRATEGY;
            std::string ads_auto_status = this->ads_engaged_ ? "Engaged" : "Not Engaged";
            std::string ads_health_status = ads_system_alert_type_ ;
            std::string params = boost::str(boost::format("vin_number:%s,license_plate:%s,carrier_name:%s,carrier_id:%s,weight:%d,ads_software_version:%s,date_of_last_state_inspection:%s,date_of_last_ads_calibration:%s,pre_trip_ads_health_check:%s,ads_health_status:%s,ads_auto_status:%s,iss_score:%d,permit_required:%s")
                                                         % vin_number_ % license_plate_ % carrier_name_ % carrier_id_ % weight_ % ads_software_version_ % date_of_last_state_inspection_ % date_of_last_ads_calibration_ % pre_trip_ads_health_check_ % ads_health_status % ads_auto_status % iss_score_ % permit_required_);
            long time = (long)(ros::Time::now().toNSec() / pow(10, 6));
            mo_msg.header.timestamp = time;
            mo_msg.strategy_params = params;
            mo_pub_.publish(mo_msg);
        }
    }    
    void TruckInspectionClient::versionCallback(const std_msgs::StringConstPtr& msg)
    {
        this->ads_software_version_ = msg->data;
    }

}
