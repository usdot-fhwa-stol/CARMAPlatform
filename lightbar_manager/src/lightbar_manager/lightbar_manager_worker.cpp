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

#include "lightbar_manager/lightbar_manager_worker.hpp"
#include <algorithm>
#include <ros/console.h>
namespace lightbar_manager
{
    LightBarManagerWorker::LightBarManagerWorker(std::string node_name) : node_name_(node_name){};

    void LightBarManagerWorker::next(const LightBarEvent& event)
    {
        lbsm_.next(event);
    }

    LightBarState LightBarManagerWorker::getCurrentState()
    {
        return lbsm_.getCurrentState();
    }

    std::vector<cav_msgs::LightBarIndicator> LightBarManagerWorker::getMsg(std::vector<LightBarIndicator> indicators)
    {
        std::vector<cav_msgs::LightBarIndicator> return_msg;
        for (auto indicator : indicators)
        {
            cav_msgs::LightBarIndicator msg;
            msg.indicator = indicator;
            return_msg.push_back(msg);
        }
        return return_msg;
    }

    std::vector<cav_msgs::LightBarCDAType> LightBarManagerWorker::getMsg(std::vector<LightBarCDAType> cda_types)
    {
        std::vector<cav_msgs::LightBarCDAType> return_msg;
        for (auto cda_type : cda_types)
        {
            cav_msgs::LightBarCDAType msg;
            msg.type = cda_type;
            return_msg.push_back(msg);
        }
        return return_msg;
    }
    cav_msgs::LightBarIndicatorControllers LightBarManagerWorker::getMsg(std::map<LightBarIndicator, std::string> ind_ctrl_map)
    {
        cav_msgs::LightBarIndicatorControllers curr;
        curr.green_solid_owner = ind_ctrl_map[GREEN_SOLID];
        curr.green_flash_owner = ind_ctrl_map[GREEN_FLASH];
        curr.yellow_sides_owner= ind_ctrl_map[YELLOW_SIDES];
        curr.yellow_dim_owner = ind_ctrl_map[YELLOW_DIM];
        curr.yellow_flash_owner = ind_ctrl_map[YELLOW_FLASH];
        curr.yellow_arrow_left_owner = ind_ctrl_map[YELLOW_ARROW_LEFT];
        curr.yellow_arrow_right_owner = ind_ctrl_map[YELLOW_ARROW_RIGHT];
        curr.yellow_arrow_out_owner = ind_ctrl_map[YELLOW_ARROW_OUT];

        return curr;
    }

    void LightBarManagerWorker::handleStateChange(const cav_msgs::GuidanceStateConstPtr& msg_ptr)
    {
        lbsm_.handleStateChange(msg_ptr);
        return;
    }

    std::vector<lightbar_manager::LightBarIndicator> LightBarManagerWorker::handleTurnSignal(const automotive_platform_msgs::TurnSignalCommandPtr& msg_ptr)
    {
        std::vector<lightbar_manager::LightBarIndicator> turn_signal;
        if (msg_ptr->turn_signal == current_turn_signal_)
        {
            return {};
        }
        if (msg_ptr->turn_signal == automotive_platform_msgs::TurnSignalCommand::LEFT) //NONE -> LEFT
        {
            turn_signal.push_back(lightbar_manager::LightBarIndicator::YELLOW_ARROW_LEFT);  
        }
        else if (msg_ptr->turn_signal == automotive_platform_msgs::TurnSignalCommand::RIGHT) //NONE -> RIGHT
        {
            turn_signal.push_back(lightbar_manager::LightBarIndicator::YELLOW_ARROW_RIGHT);
        }
        else if (msg_ptr->turn_signal == automotive_platform_msgs::TurnSignalCommand::NONE) 
        {
            // check previous signal
            if (current_turn_signal_ == automotive_platform_msgs::TurnSignalCommand::RIGHT) // RIGHT -> NONE
                turn_signal.push_back(lightbar_manager::LightBarIndicator::YELLOW_ARROW_RIGHT);
            else if (current_turn_signal_ == automotive_platform_msgs::TurnSignalCommand::LEFT) // LEFT -> NONE
                turn_signal.push_back(lightbar_manager::LightBarIndicator::YELLOW_ARROW_LEFT);
        }

        current_turn_signal_ = msg_ptr->turn_signal;

        return turn_signal;
    }
    std::map<LightBarIndicator, std::string> LightBarManagerWorker::getIndicatorControllers()
    {
        return ind_ctrl_map_;
    }
    

    std::map<LightBarCDAType, LightBarIndicator> LightBarManagerWorker::setIndicatorCDAMap(std::map<std::string, std::string> raw_map)
    {
        // In case if the parameter is not loaded corretly and there is an error, return this`1
        std::map<LightBarCDAType, LightBarIndicator> default_map;
        
        default_map[TYPE_A] = YELLOW_DIM;
        default_map[TYPE_B] = YELLOW_DIM;
        default_map[TYPE_C] = YELLOW_FLASH;
        default_map[TYPE_D] = YELLOW_SIDES;

        if (raw_map.size() < 4)
        {
            ROS_WARN_STREAM("In function: " << __FUNCTION__ << ": LightBarManager's CDAType to Indicator table is not configured correctly. Using default mapping...");
            cda_ind_map_ = default_map;
            return cda_ind_map_;
        }
        
        // Convert from string:string mapping to correct enums
        for (std::pair<std::string, std::string> element : raw_map)
        {
            LightBarCDAType cda_type;
            LightBarIndicator indicator;
            try
            {
                cda_type = cda_type_dict_.at(element.first);
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM ("In function: " << __FUNCTION__ << 
                ": LightBarManager Received unknown CDA Msg Type. Using default mapping for cda-indicators...");
                cda_ind_map_ = default_map;
                return cda_ind_map_;
            }
            try
            {
                indicator = ind_dict.at(element.second);
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM ("In function: " << __FUNCTION__ << 
                ": LightBarManager Received unknown indicator type. Using default mapping for cda-indicators...");
                cda_ind_map_ = default_map;
                return cda_ind_map_;
            }
            cda_ind_map_.emplace(cda_type, indicator);
        }
        return cda_ind_map_;
    }

    bool LightBarManagerWorker::hasHigherPriority (std::string requester, std::string controller)
    {
        auto start = control_priorities.begin();
        auto end = control_priorities.end();
        
        auto requesterPriority = std::find(start, end, requester);
        auto controllerPriority = std::find(start, end, controller);

        // Components not in the priority list are assumed to have the lowest priority
        if (requesterPriority == end)
        {
            ROS_WARN_STREAM(requester << " is referenced in lightbar_manager, but is not in the priority list");
            return false;
        }  
        else if (controllerPriority == end)
        {
            ROS_WARN_STREAM(controller << " is referenced in lightbar_manager and is controlling an indicator, but is not in the priority list");
            return true;
        }
        return (requesterPriority - end) <= (controllerPriority - end);
    }

    std::vector<LightBarIndicator> LightBarManagerWorker::requestControl(std::vector<LightBarIndicator> ind_list, std::string requester_name)
    {
        std::vector<LightBarIndicator> denied_list;
        std::string indicator_owner;
        // Attempt to acquire control of every indicators
        for (LightBarIndicator indicator: ind_list) 
        {
            // Attempt control
            try
            {
                indicator_owner = ind_ctrl_map_.at(indicator);
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM("In function: " << __FUNCTION__ << ", the component, " << requester_name 
                    << ", requested a control of invalid indicator. Skipping with WARNING:" << e.what() << "\n");
                continue;
            }
        
            if (indicator_owner == "") 
            {   
                // Add new controller If no other component has claimed this indicator
                handleControlChange(indicator, requester_name, CONTROL_GAINED);
            } 
            else if (indicator_owner != requester_name) 
            {   
                // If this indicator is already controlled
                // If the requesting component has higher priority it may take control of this indicator
                if (hasHigherPriority(requester_name, indicator_owner)) 
                {
                    // Handle previous controller
                    handleControlChange(indicator, indicator_owner, CONTROL_LOST);
                    // Add new controller
                    handleControlChange(indicator, requester_name, CONTROL_GAINED);
                } 
                else 
                {
                    denied_list.push_back(indicator); // Notify caller of failure to take control of component
                }
            }
        }
        return denied_list;
    }   

    void LightBarManagerWorker::releaseControl(std::vector<LightBarIndicator> ind_list, std::string owner_name)
    {
        std::string current_owner;
        
        // Attempt to release control of all indicators
        for (LightBarIndicator indicator: ind_list) 
        {
            // Attempt release control
            try
            {
                current_owner = ind_ctrl_map_.at(indicator);
            }
            catch(const std::exception& e)
            {
                ROS_WARN_STREAM("In function: " << __FUNCTION__ << ", the component, " << owner_name 
                    << ", requested a release of an invalid indicator. Skipping with WARNING:" << e.what() << "\n");
                continue;
            }

            // Lose control only if the requester is currently controlling it
            if (current_owner == owner_name) 
            {   
                handleControlChange(indicator,owner_name, CONTROL_LOST);
            }
        }

        return;
    }
    
    void LightBarManagerWorker::handleControlChange(LightBarIndicator indicator, std::string controller, IndicatorControlEvent event)
    {
        // Pick new owner name depending on losing or gaining control
        std::string new_owner;
        if (event == CONTROL_GAINED)
            new_owner = controller;
        else
            new_owner = "";
        
        // Handle mutually in-exclusive indicators
        // These are indicators that are controlled indirectly due to change in one indicator
        // Priority of every one of those indicators, must be compared to that of requester to change it.
        // e.g. if A>B and YELLOW_RIGHT= "" & YELLOW_LEFT = A (means YELLOW_FLASH = A & YELLOW_OUT = A)
        // Then if YELLOW_RIGHT = "" -> YELLOW_RIGHT = "B", B should not be able to control YELLOW_FLASH nor YELLOW_OUT
        switch (indicator)
        {
            case YELLOW_ARROW_LEFT:
            case YELLOW_ARROW_RIGHT:
                ind_ctrl_map_[YELLOW_ARROW_OUT] = 
                    hasHigherPriority(controller, ind_ctrl_map_[YELLOW_ARROW_OUT]) ? new_owner :ind_ctrl_map_[YELLOW_ARROW_OUT];
                ind_ctrl_map_[YELLOW_FLASH] = ind_ctrl_map_[YELLOW_ARROW_OUT]; //they always have same owner
                ind_ctrl_map_[indicator] = new_owner;
                break;
            case YELLOW_ARROW_OUT:
            case YELLOW_FLASH:
                ind_ctrl_map_[YELLOW_ARROW_LEFT] = 
                    hasHigherPriority(controller, ind_ctrl_map_[YELLOW_ARROW_LEFT]) ? new_owner :ind_ctrl_map_[YELLOW_ARROW_LEFT];
                ind_ctrl_map_[YELLOW_ARROW_RIGHT] = 
                    hasHigherPriority(controller, ind_ctrl_map_[YELLOW_ARROW_RIGHT]) ? new_owner :ind_ctrl_map_[YELLOW_ARROW_RIGHT];
                ind_ctrl_map_[YELLOW_ARROW_OUT] = 
                    hasHigherPriority(controller, ind_ctrl_map_[YELLOW_ARROW_OUT]) ? new_owner :ind_ctrl_map_[YELLOW_ARROW_OUT];
                ind_ctrl_map_[YELLOW_FLASH] = ind_ctrl_map_[YELLOW_ARROW_OUT]; //they always have same owner
                break;
            case GREEN_FLASH:
            case GREEN_SOLID:
                ind_ctrl_map_[GREEN_FLASH] = 
                    hasHigherPriority(controller, ind_ctrl_map_[GREEN_FLASH]) ? new_owner :ind_ctrl_map_[GREEN_FLASH];
                ind_ctrl_map_[GREEN_SOLID] = ind_ctrl_map_[GREEN_FLASH]; //they always have same owner
                break;
            default:
                ind_ctrl_map_[indicator] = new_owner;
                break;
        }
        return;
    }

    std::vector<IndicatorStatus> LightBarManagerWorker::setIndicator(LightBarIndicator ind, IndicatorStatus ind_status, std::string requester_name)
    {
        std::string current_controller = ind_ctrl_map_[ind];

        // Use a local copy in case manager fails to set the light
        std::vector<IndicatorStatus> light_status_copy = light_status;

        // Handle mutually non-exclusive cases
        // If desired indicator is already at the status do not change any indicators 
        if (ind_status != light_status_copy[ind])
        {
            switch(ind)
            {
                case YELLOW_ARROW_LEFT:
                case YELLOW_ARROW_RIGHT:
                    light_status_copy[YELLOW_ARROW_OUT] = OFF;
                    light_status_copy[YELLOW_FLASH] = OFF;
                    break;
                case YELLOW_ARROW_OUT:
                case YELLOW_FLASH:
                    light_status_copy[YELLOW_ARROW_OUT] = OFF;
                    light_status_copy[YELLOW_FLASH] = OFF;
                    light_status_copy[YELLOW_ARROW_LEFT] = OFF;
                    light_status_copy[YELLOW_ARROW_RIGHT] = OFF;
                    break;
                case GREEN_FLASH:
                case GREEN_SOLID:
                    light_status_copy[GREEN_FLASH] = OFF;
                    light_status_copy[GREEN_SOLID] = OFF;
                    break;
                default:
                    break;
            }
        }
        
        // Set the desired indicator now that there is no conflict.
        light_status_copy[ind] = ind_status;
        return light_status_copy;

    }

    cav_msgs::LightBarStatus LightBarManagerWorker::getLightBarStatusMsg(std::vector<IndicatorStatus> indicators)
    {
        // it is assumed that mutually exclusive cases are handled properly.
        cav_msgs::LightBarStatus msg;
        msg.green_solid = indicators[GREEN_SOLID] == ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        msg.green_flash = indicators[GREEN_FLASH]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        msg.sides_solid = indicators[YELLOW_SIDES]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        msg.yellow_solid = indicators[YELLOW_DIM]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        msg.flash = indicators[YELLOW_FLASH]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;

        msg.left_arrow = indicators[YELLOW_ARROW_LEFT]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        msg.right_arrow = indicators[YELLOW_ARROW_RIGHT]== ON ? cav_msgs::LightBarStatus::ON : cav_msgs::LightBarStatus::OFF;
        // for YELLOW_ARROW_OUT set left and right
        if (indicators[YELLOW_ARROW_OUT] == ON)
        {
            msg.left_arrow = cav_msgs::LightBarStatus::ON;
            msg.right_arrow = cav_msgs::LightBarStatus::ON;
        }
        return msg;
    }

    void LightBarManagerWorker::setIndicatorControllers()
    {
        for (int i = 0; i <= INDICATOR_COUNT; i++ )
        {
            LightBarIndicator indicator = static_cast<LightBarIndicator>(i);
            // initialize the owner as empty string
            ind_ctrl_map_[indicator] = "";
        }
    }

    void LightBarManagerWorker::setIndicatorControllers(std::map<LightBarIndicator, std::string> ind_ctrl_map)
    {
        ind_ctrl_map_ = ind_ctrl_map;
    }

    LightBarIndicator LightBarManagerWorker::getIndicatorFromCDAType(LightBarCDAType cda_type)
    {
        return cda_ind_map_[cda_type];
    }

    LightBarCDAType LightBarManagerWorker::getCDATypeFromIndicator(LightBarIndicator indicator)
    {
        for (std::pair<LightBarCDAType,LightBarIndicator> element : cda_ind_map_) 
        {
            if (element.second == indicator)
                return element.first;
	    }
        // if the indicator does not have any mapped CDA Msg Type, throw and handle outside
        throw INDICATOR_NOT_MAPPED(std::string("Specified indicator does not have a mapped CDA Msg Type. Skipping...")); 
    }

}
