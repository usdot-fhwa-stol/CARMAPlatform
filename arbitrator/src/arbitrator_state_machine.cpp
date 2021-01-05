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

#include "arbitrator_state_machine.hpp"

namespace arbitrator
{

    ArbitratorState ArbitratorStateMachine::get_state()
    {
        return current_state;        
    }


    ArbitratorState ArbitratorStateMachine::submit_event(ArbitratorEvent event)
    {
        for (auto iter = ARBITRATOR_TRANSITIONS.begin(); iter != ARBITRATOR_TRANSITIONS.end(); iter++) 
        {
            if (current_state == iter->current_state && event == iter->input_event) {
                // In the event of multiple legal transistions this will take the last transition,
                // But ultimately this condition should never arise.
                current_state = iter->final_state;
                break;
            }
        }

        return current_state;
    }

};