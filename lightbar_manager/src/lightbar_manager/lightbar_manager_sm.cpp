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

#include "lightbar_manager/lightbar_manager_sm.hpp"

namespace lightbar_manager
{
// Default constructor
LightBarManagerStateMachine::LightBarManagerStateMachine() {}


void LightBarManagerStateMachine::next(const LightBarEvent& event)
{
    // Set to DISENGAGED state if guidance is disingaged
    if(event == LightBarEvent::GUIDANCE_DISENGAGED) {
        current_state_ = DISENGAGED;
        
        return;
    }

    switch(current_state_)
    {
        case DISENGAGED:
            if(event == LightBarEvent::GUIDANCE_CONNECTED) {
                current_state_ = ACTIVE;
                
            }
            else if(event == LightBarEvent::GUIDANCE_ENGAGED) {
                current_state_ = ENGAGED;
                                    
            }
            break;
        case ENGAGED:
            if(event == LightBarEvent::GUIDANCE_DISCONNECTED) {
                current_state_ = ACTIVE;
                
            }
            break;
        case ACTIVE:
            if(event == LightBarEvent::GUIDANCE_ENGAGED) {
                current_state_ = ENGAGED;
            }
            break;
        default:
            break;
    }
}

void LightBarManagerStateMachine::handleStateChange(const cav_msgs::GuidanceStateConstPtr& msg_ptr)
{
    // Check if the msg is as same as before
    if (msg_ptr->state == guidance_state_)
        return;
    
    switch (msg_ptr->state)
    {
        case cav_msgs::GuidanceState::STARTUP:
        case cav_msgs::GuidanceState::SHUTDOWN:
            onDisengage();
            break;

        case cav_msgs::GuidanceState::ACTIVE:
        case cav_msgs::GuidanceState::DRIVERS_READY:
        case cav_msgs::GuidanceState::INACTIVE:
            onActive();
            break;

        case cav_msgs::GuidanceState::ENGAGED:
            onEngage();
            break;

        default:
            ROS_WARN_STREAM("LightBarManager received unknown state from guidance state machine:" << msg_ptr->state);
            break;
    }
    // Update the current state
    guidance_state_ = msg_ptr->state;
}

void LightBarManagerStateMachine::onDisengage()
{
    next(GUIDANCE_DISENGAGED);
}

void LightBarManagerStateMachine::onActive()
{
    // Transitioning FROM ENGAGED state to Active
    if (guidance_state_ == cav_msgs::GuidanceState::ENGAGED)
        next(GUIDANCE_DISCONNECTED);
    // Transitioning FROM all "OFF" states of guidance for lightbar (STARTUP, DRIVERS_READY, INACTIVE, OFF)
    else
        next(GUIDANCE_CONNECTED);
}

void LightBarManagerStateMachine::onEngage()
{
    next(GUIDANCE_ENGAGED);
}


LightBarState LightBarManagerStateMachine::getCurrentState()
{
    return current_state_;
}

} // namespace lightbar_manager
