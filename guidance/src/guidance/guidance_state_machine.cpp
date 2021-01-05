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

#include "guidance/guidance_state_machine.hpp"

namespace guidance
{
    void GuidanceStateMachine::onGuidanceSignal(Signal signal)
    {
        // Set to OFF state of SHUTDOWN signal received
        if(signal == Signal::SHUTDOWN) {
            current_guidance_state_ = State::OFF;
            return;
        }
        switch(current_guidance_state_)
        {
            case State::STARTUP:
                if(signal == Signal::INITIALIZED) {
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            case State::DRIVERS_READY:
                if(signal == Signal::ACTIVATED)
                {
                    current_guidance_state_ = State::ACTIVE;
                }
                break;
            case State::ACTIVE:
                if(signal == Signal::ENGAGE)
                {
                    current_guidance_state_ = State::ENGAGED;
                } else if(signal == Signal::DISENGAGED)
                {
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            case State::ENGAGED:
                if(signal == Signal::DISENGAGED)
                {
                    current_guidance_state_ = State::DRIVERS_READY;
                } else if(signal == Signal::OVERRIDE)
                {
                    current_guidance_state_ = State::INACTIVE;
                }
                break;
            case State::INACTIVE:
                if(signal == Signal::ACTIVATED)
                {
                    current_guidance_state_ = State::ACTIVE;
                }
                break;
            default:
                break;
        }
    }

    void GuidanceStateMachine::onSystemAlert(const cav_msgs::SystemAlertConstPtr& msg)
    {
        if(msg->type == msg->DRIVERS_READY)
        {
            onGuidanceSignal(Signal::INITIALIZED);
        } else if(msg->type == msg->SHUTDOWN || msg->type == msg->FATAL)
        {
            onGuidanceSignal(Signal::SHUTDOWN);
        }
    }

    void GuidanceStateMachine::onSetGuidanceActive(bool msg)
    {
        if(msg)
        {
            onGuidanceSignal(Signal::ACTIVATED);
        } else
        {
            onGuidanceSignal(Signal::DISENGAGED);
        }
        
    }

    void GuidanceStateMachine::onRoboticStatus(const cav_msgs::RobotEnabledConstPtr& msg)
    {
        // robotic status changes from false to true
        if(!robotic_active_status_ && msg->robot_active)
        {
            robotic_active_status_ = true;
            onGuidanceSignal(Signal::ENGAGE);
        }
        // robotic status changes from true to false
        else if(robotic_active_status_ && !msg->robot_active)
        {
            robotic_active_status_ = false;
            onGuidanceSignal(Signal::OVERRIDE);
        }
    }

    void GuidanceStateMachine::onRouteEvent(const cav_msgs::RouteEventConstPtr& msg)
    {
        if(msg->event == cav_msgs::RouteEvent::ROUTE_DEPARTED ||
           msg->event == cav_msgs::RouteEvent::ROUTE_COMPLETED ||
           msg->event == cav_msgs::RouteEvent::ROUTE_ABORTED) {
               onGuidanceSignal(Signal::DISENGAGED);
           }
    }

    uint8_t GuidanceStateMachine::getCurrentState()
    {
        return static_cast<uint8_t>(current_guidance_state_);
    }

    bool GuidanceStateMachine::shouldCallSetEnableRobotic()
    {
        // call SetEnableRobotic service once when we enter ACTIVE state
        if(current_guidance_state_ == GuidanceStateMachine::ACTIVE)
        {
            if(!called_robotic_engage_in_active_)
            {
                called_robotic_engage_in_active_ = true;
                return true;
            }
        } else {
            // Reset when we leave ACTIVE state 
            called_robotic_engage_in_active_ = false;
        }
        return false;
    }

}
