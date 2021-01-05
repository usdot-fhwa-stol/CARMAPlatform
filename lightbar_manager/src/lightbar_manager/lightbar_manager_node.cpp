/*
 * Copyright (C) 2018-2021 LEIDOS.
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

#include "lightbar_manager/lightbar_manager_node.hpp"
#include <algorithm>

namespace lightbar_manager
{
LightBarManager::LightBarManager(const std::string& node_name) : 
    lbm_(std::make_shared<LightBarManagerWorker>(LightBarManagerWorker(node_name))),
    node_name_ (node_name) {}

void LightBarManager::turnOffAll()
{
    // All components lose controls and turn the indicators off by giving lightbarmanar the control
    std::vector<LightBarIndicator> all_indicators;

    ROS_INFO_STREAM("LightBarManager was commanded to turn off all indicators!");

    for (std::pair <LightBarIndicator, std::string> element : lbm_->getIndicatorControllers())
        all_indicators.push_back(element.first);
    
    // Reset all controls
    lbm_->setIndicatorControllers();
    
    // Give lightbar manager the control which is guaranteed to succeed
    lbm_->requestControl(all_indicators, node_name_);
    int response_code = 0;
    for (auto indicator : all_indicators)
    {
        response_code = setIndicator(indicator, OFF, node_name_);
        if (response_code != 0)
            ROS_WARN_STREAM ("In Function " << __FUNCTION__ << ": LightBarManager was not able to turn off indicator ID:" 
                << indicator << ". Response code: " << response_code);
    }

    // Once forced them off, release controls in case carma is still running so that lightbar_manager does not hog the control
    all_indicators.erase(std::remove(all_indicators.begin(), all_indicators.end(), GREEN_FLASH), all_indicators.end());
    all_indicators.erase(std::remove(all_indicators.begin(), all_indicators.end(), GREEN_SOLID), all_indicators.end());
    lbm_->releaseControl(all_indicators, node_name_);
}

bool LightBarManager::requestControlCallBack(cav_srvs::RequestIndicatorControlRequest& req, cav_srvs::RequestIndicatorControlResponse& res)
{
    std::vector<LightBarIndicator> ind_list, controlled_ind_list;
    std::vector<LightBarCDAType> controlled_cda_type_list;

    // Use CDAType if the field is not empty in the request
    if (req.cda_list.size() != 0)
    {   
        for (auto cda_type : req.cda_list) 
        {
            // return false if invalid cda_type number
            if (static_cast<uint8_t>(cda_type.type) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(cda_type.type)));
        }
    }
    else
    {
        for (auto indicator : req.ind_list) 
        {
            // return false if invalid indicator number
            if (static_cast<uint8_t>(indicator.indicator) >= INDICATOR_COUNT)
                return false;
            ind_list.push_back(static_cast<LightBarIndicator>(indicator.indicator));
        }
            
    }
    
    controlled_ind_list = lbm_->requestControl(ind_list, req.requester_name);
    
    for (auto indicator : controlled_ind_list)
    {
        try
        {
            // Some indicators do not have mapped CDAType
            controlled_cda_type_list.push_back(lbm_->getCDATypeFromIndicator(indicator));
        }
        catch(INDICATOR_NOT_MAPPED)
        {
            // 4 out of 8 does not have mapping, so skip
            continue;
        }
    }
        
    // Modify the response
    res.cda_list = lbm_->getMsg(controlled_cda_type_list);
    res.ind_list = lbm_->getMsg(controlled_ind_list);
    return true;
}

bool LightBarManager::releaseControlCallBack(cav_srvs::ReleaseIndicatorControlRequest& req, cav_srvs::ReleaseIndicatorControlResponse& res)
{
    std::vector<LightBarIndicator> ind_list;

    // Use CDAType if the field is not empty in the request
    if (req.cda_list.size() != 0)
    {   
        for (auto cda_type : req.cda_list) 
        {
            // return false if invalid indicator number
            if (static_cast<uint8_t>(cda_type.type) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(cda_type.type)));
        }
    }
    else
    {
        for (auto indicator : req.ind_list)
        {
            // return false if invalid cda_type number
            if (static_cast<uint8_t>(indicator.indicator) >= INDICATOR_COUNT)
                return false; 
            ind_list.push_back(static_cast<LightBarIndicator>(indicator.indicator));
        }
    }
    lbm_->releaseControl(ind_list, req.requester_name);
    return true;
}

bool LightBarManager::spinCallBack()
{
    
    indicator_control_publisher_.publish(lbm_->getMsg(lbm_->getIndicatorControllers()));
    return true;
}

void LightBarManager::stateChangeCallBack(const cav_msgs::GuidanceStateConstPtr& msg_ptr)
{
    // Relay the msg to state machine
    LightBarState prev_lightbar_state = lbm_->getCurrentState();
    lbm_->handleStateChange(msg_ptr);

    // Change green lights depending on states, no need to check if its current owner, as it will always be for green lights
    LightBarState curr_lightbar_state = lbm_->getCurrentState();
    if (curr_lightbar_state != prev_lightbar_state)
    {
        switch(curr_lightbar_state)
        {
            case DISENGAGED:
                turnOffAll();
                break;
            case ACTIVE:
                setIndicator(GREEN_FLASH, ON, node_name_);
                break;
            case ENGAGED:
                setIndicator(GREEN_SOLID, ON, node_name_);
                break;
            default:
                break;
        }
    }
}

void LightBarManager::turnSignalCallback(const automotive_platform_msgs::TurnSignalCommandPtr& msg_ptr)
{
    // if not automated
    if (msg_ptr->mode != 1)
    {
        return;
    }

    // check if left or right signal should be controlled, or none at all
    std::vector<lightbar_manager::LightBarIndicator> changed_turn_signal = lbm_->handleTurnSignal(msg_ptr);

    if (changed_turn_signal.empty())
    {
        return; //no need to do anything if it is same turn signal changed
    }
    
    lightbar_manager::IndicatorStatus indicator_status;
    // check if we should turn off or on given any indicator
    if (msg_ptr->turn_signal == automotive_platform_msgs::TurnSignalCommand::NONE)
    {
        indicator_status = lightbar_manager::IndicatorStatus::OFF;
    }
    else 
    {
        indicator_status = lightbar_manager::IndicatorStatus::ON;
        prev_owners_before_turn_ = lbm_->getIndicatorControllers(); // save the owner if new turn is starting
    }

    if (lbm_->requestControl(changed_turn_signal, node_name_).empty())
    {
        int response_code = 0;
        response_code = setIndicator(changed_turn_signal[0], indicator_status, node_name_);
        if (response_code != 0)
            ROS_ERROR_STREAM ("In Function " << __FUNCTION__ << ": LightBarManager was not able to set light of indicator ID:" 
                << changed_turn_signal[0] << ". Response code: " << response_code);
    }
    else
    {
        std::string turn_string = msg_ptr->turn_signal == automotive_platform_msgs::TurnSignalCommand::LEFT ? "left" : "right";
        ROS_WARN_STREAM("Lightbar was not able to take control of lightbar to indicate " << turn_string << "turn!");
        return;
    }

    // release control if it is turning off and put back the previous owners
    if (indicator_status == lightbar_manager::IndicatorStatus::OFF)
    {
        lbm_->releaseControl(changed_turn_signal, node_name_);
        // we need to force set the <indicator, owner> mapping
        // as series of requestControl may not be able to exactly achieve previous combination
        lbm_->setIndicatorControllers(prev_owners_before_turn_);
        // reset as the turn is finished
        prev_owners_before_turn_.clear();
    }
}

std::shared_ptr<LightBarManagerWorker> LightBarManager::getWorker()
{
    return lbm_;
}

int LightBarManager::setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, const std::string& requester_name)
{
    // Handle the msg/service translation
    int response_code = 0;
    if (static_cast<uint8_t>(ind) >= INDICATOR_COUNT || static_cast<uint8_t>(ind_status) > 1)
    {
        // Invalid indicator ID or ind_status
        response_code = 1;
        return response_code;
    }

    // Check if the requester has control of this light
    std::string current_controller = lbm_->getIndicatorControllers()[ind];
    if (requester_name == "" || current_controller != requester_name) 
    {
        ROS_WARN_STREAM(requester_name << " failed to set the LightBarIndicator ID" << ind 
            << " as this was already controlled by " << current_controller);
        response_code = 1;
        return response_code;
    }

    std::vector<IndicatorStatus> light_status_proposed = lbm_->setIndicator(ind, ind_status, requester_name);
    cav_msgs::LightBarStatus msg = lbm_->getLightBarStatusMsg(light_status_proposed);
    cav_srvs::SetLights srv;
    srv.request.set_state = msg;
    
    // Try to send the request
    if (lightbar_driver_client_.call(srv))
    {
        // if successful, update the local copy.
        lbm_->light_status = light_status_proposed;
        response_code =  0;
    }
    else
    {
        ROS_WARN_STREAM("In function: " << __FUNCTION__ << ": Failed to set lights. ROSservice call to the driver failed. ");
        response_code =  2;
    }
    return response_code;

}
bool LightBarManager::setIndicatorCallBack(cav_srvs::SetLightBarIndicatorRequest& req, cav_srvs::SetLightBarIndicatorResponse& res)
{
    LightBarIndicator indicator;
    int response_code = 0;
    if (req.cda_type.type != NULL)
        indicator = lbm_->getIndicatorFromCDAType(static_cast<LightBarCDAType>(req.cda_type.type));
    else
        indicator = static_cast<LightBarIndicator>(req.indicator.indicator);

    response_code = setIndicator(indicator, static_cast<IndicatorStatus>(req.state), req.requester_name);
    res.status_code = response_code;
    if (response_code != 0)
        return false;

    return true;
}

void LightBarManager::init(std::string mode)
{
    ROS_INFO("Initalizing lightbar manager node...");
    
    // Load the spin rate param to determine how fast to process messages
    // Default rate 10.0 Hz
    spin_rate_ = pnh_.param<double>("spin_rate_hz", 10.0);
    std::string lightbar_driver_service_name= pnh_.param<std::string>("lightbar_driver_service_name", "set_lights");
    std::string guidance_state_topic_name = pnh_.param<std::string>("guidance_state_topic_name", "state");
    std::string turn_signal_topic_name = pnh_.param<std::string>("turn_signal_topic_name", "turn_signal_command");
    // Init our ROS objects
    request_control_server_= nh_.advertiseService("request_control", &LightBarManager::requestControlCallBack, this);
    release_control_server_= nh_.advertiseService("release_control", &LightBarManager::releaseControlCallBack, this);
    set_indicator_server_= nh_.advertiseService("set_indicator", &LightBarManager::setIndicatorCallBack, this);
    indicator_control_publisher_ = nh_.advertise<cav_msgs::LightBarIndicatorControllers>("indicator_control", 5);
    guidance_state_subscriber_ = nh_.subscribe(guidance_state_topic_name, 5, &LightBarManager::stateChangeCallBack, this);
    turn_signal_subscriber_ = nh_.subscribe(turn_signal_topic_name, 5, &LightBarManager::turnSignalCallback, this);
    lightbar_driver_client_ = nh_.serviceClient<cav_srvs::SetLights>(lightbar_driver_service_name);

    // Load Conversion table, CDAType to Indicator mapping
    std::map<std::string,std::string> cda_ind_map_raw;
    pnh_.getParam("lightbar_cda_to_ind_table", cda_ind_map_raw);
    lbm_->setIndicatorCDAMap(cda_ind_map_raw);

    // Initialize indicator control map. Fills with supporting indicators with empty string name as owners.
    lbm_->setIndicatorControllers();

    // Initialize indicator representation of lightbar status to all OFF
    for (int i =0; i < INDICATOR_COUNT; i++)
        lbm_->light_status.push_back(OFF);

    // Load lightbar priorities. 
    pnh_.getParam("lightbar_priorities", lbm_->control_priorities);

    // Setup priorities for unit test 
    if (mode == "test")
        setupUnitTest();

    // Take control of green light
    bool normal_operation = pnh_.param<bool>("normal_operation", true);
    std::vector<LightBarIndicator> denied_list, greens = {GREEN_SOLID, GREEN_FLASH};
    denied_list = lbm_->requestControl(greens, node_name_);
    if (denied_list.size() != 0)
    {
        if (normal_operation)
        {
            ROS_WARN_STREAM("In fuction " << __FUNCTION__ << ", LightBarManager was not able to take control of all green indicators."
                << ".\n Please check priority list rosparameter, and ensure " << node_name_ << " has the highest priority."
                << ".\n If this is intended, pass false to normal_operation argument. Exiting...");
            throw INVALID_LIGHTBAR_MANAGER_PRIORITY();
        }
        else
        {
            ROS_WARN_STREAM("In fuction " << __FUNCTION__ << ", LightBarManager was not able to take control of all green indicators."
                << ".\n Resuming...");   
        }
    }
    return;
}

void LightBarManager::setupUnitTest()
{
    // Add mock components for unit test
    while (lbm_->control_priorities.size() != 0)
        lbm_->control_priorities.pop_back();
    lbm_->control_priorities.push_back("lightbar_manager");
    lbm_->control_priorities.push_back("tester1");
    lbm_->control_priorities.push_back("tester2");
    lbm_->control_priorities.push_back("tester3");
    return;
}

int LightBarManager::run()
{
    // Initialize
    init();

    // Spin until system shutdown
    ROS_INFO_STREAM("LightBarManager node initialized, spinning at " << spin_rate_ << "hz...");
    ros::CARMANodeHandle::setSpinCallback(std::bind(&LightBarManager::spinCallBack, this));
    ros::CARMANodeHandle::setSpinRate(spin_rate_);
    ros::CARMANodeHandle::spin();

    // return
    return 0;
} 
} // namespace lightbar_manager
