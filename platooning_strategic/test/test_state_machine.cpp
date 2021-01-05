
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

#include "state_machine.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include "mobility_messages.cpp"
// #include "TestHelpers.h"



TEST(PlatooningStateMachineTest, test1)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;
    

    psm.pm_ = pm;


    psm.current_platoon_state = platoon_strategic::PlatoonState::FOLLOWER;

    MobilityMessages mm;
    cav_msgs::MobilityOperation res1 = mm.getOperation1();

    psm.onMobilityOperationMessage(res1);


    
}

TEST(PlatooningStateMachineTest, test2)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;


    psm.current_platoon_state = platoon_strategic::PlatoonState::LEADER;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest1();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);



    EXPECT_EQ(2, out);

    
}

TEST(PlatooningStateMachineTest, test3)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;
    psm.applicantID = "11";

    psm.current_platoon_state = platoon_strategic::PlatoonState::LEADERWAITING;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest2();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);


    EXPECT_EQ(2, psm.current_platoon_state);
    EXPECT_EQ(0, out);

    
}

TEST(PlatooningStateMachineTest, test4)
{
    std::vector<platoon_strategic::PlatoonMember> cur_pl;
    
    platoon_strategic::PlatooningStateMachine psm;
    platoon_strategic::PlatoonManager pm;

    psm.pm_ = pm;
    psm.applicantID = "11";

    psm.current_platoon_state = platoon_strategic::PlatoonState::CANDIDATEFOLLOWER;

    MobilityMessages mm;
    cav_msgs::MobilityRequest r1 = mm.getRequest1();

    platoon_strategic::MobilityRequestResponse out = psm.onMobilityRequestMessage(r1);

    // cav_msgs::MobilityRequest r1;
    // psm.onMobilityRequestMessage(r1);

    // cav_msgs::MobilityResponse r1;
    // psm.onMobilityResponseMessage(r1);

    EXPECT_EQ(2, out);
}

