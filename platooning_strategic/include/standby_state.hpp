#pragma once

#include "state_machine.hpp"

namespace platoon_strategic
{
    class StandbyState: public PlatooningStateMachine
    {
    public:
        StandbyState() {};

        MobilityRequestResponse onMobilityRequestMessage(cav_msgs::MobilityRequest &msg);
        void onMobilityResponseMessage(cav_msgs::MobilityResponse &msg);
        void onMobilityOperationMessage(cav_msgs::MobilityOperation &msg);
        cav_msgs::Maneuver planManeuver(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time current_time);


    };
}