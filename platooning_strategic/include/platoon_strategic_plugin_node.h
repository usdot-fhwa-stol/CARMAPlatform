#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityOperation.h>
#include <cav_msgs/PlatooningInfo.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include "platoon_strategic.hpp"
#include "platoon_config.h"

namespace platoon_strategic
{
/**
 * \brief ROS node for the YieldPlugin
 */ 
class PlatoonStrategicPluginNode
{
public:

  /**
   * \brief Entrypoint for this node
   */ 
  void run()
  {
    ros::CARMANodeHandle nh;
    ros::CARMANodeHandle pnh("~");

    carma_wm::WMListener wml;
    auto wm_ = wml.getWorldModel();

    ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
    ros::Publisher mob_response_pub = nh.advertise<cav_msgs::MobilityResponse>("outgoing_mobility_response", 1);
    ros::Publisher mob_request_pub = nh.advertise<cav_msgs::MobilityRequest>("outgoing_mobility_request", 1);
    ros::Publisher mob_operation_pub = nh.advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 1);
    ros::Publisher platoon_info_pub = nh.advertise<cav_msgs::PlatooningInfo>("platoon_info", 1);


    PlatoonPluginConfig config;

    pnh.param<int>("maxPlatoonSize", config.maxPlatoonSize, config.maxPlatoonSize);
    pnh.param<int>("algorithmType", config.algorithmType, config.algorithmType);
    pnh.param<int>("statusMessageInterval", config.statusMessageInterval, config.statusMessageInterval);
    pnh.param<int>("infoMessageInterval", config.infoMessageInterval, config.infoMessageInterval);
    pnh.param<double>("timeHeadway", config.timeHeadway, config.timeHeadway);
    pnh.param<double>("standStillHeadway", config.standStillHeadway, config.standStillHeadway);
    pnh.param<double>("maxAllowedJoinTimeGap", config.maxAllowedJoinTimeGap, config.maxAllowedJoinTimeGap);
    pnh.param<double>("maxAllowedJoinGap", config.maxAllowedJoinGap, config.maxAllowedJoinGap);
    pnh.param<double>("desiredJoinTimeGap", config.desiredJoinTimeGap, config.desiredJoinTimeGap);
    pnh.param<double>("desiredJoinGap", config.desiredJoinGap, config.desiredJoinGap);
    pnh.param<double>("waitingStateTimeout", config.waitingStateTimeout, config.waitingStateTimeout);
    pnh.param<double>("cmdSpeedMaxAdjustment", config.cmdSpeedMaxAdjustment, config.cmdSpeedMaxAdjustment);
    pnh.param<double>("lowerBoundary", config.lowerBoundary, config.lowerBoundary);
    pnh.param<double>("upperBoundary", config.upperBoundary, config.upperBoundary);
    pnh.param<double>("maxSpacing", config.maxSpacing, config.maxSpacing);
    pnh.param<double>("minSpacing", config.minSpacing, config.minSpacing);
    pnh.param<double>("minGap", config.minGap, config.minGap);
    pnh.param<double>("maxGap", config.maxGap, config.maxGap);
    pnh.param<double>("maxCrosstrackError", config.maxCrosstrackError, config.maxCrosstrackError);
    pnh.getParam("/vehicle_length", config.vehicleLength);
    pnh.getParam("/vehicle_id", config.vehicle_id);
    
    ROS_INFO_STREAM("PlatoonPluginConfig Params" << config);

    PlatoonStrategicPlugin worker(wm_, config, [&discovery_pub](auto msg) { discovery_pub.publish(msg); }, [&mob_response_pub](auto msg) { mob_response_pub.publish(msg); },
                                    [&mob_request_pub](auto msg) { mob_request_pub.publish(msg); }, [&mob_operation_pub](auto msg) { mob_operation_pub.publish(msg); },
                                    [&platoon_info_pub](auto msg) { platoon_info_pub.publish(msg); } );
  
    // worker.lookupECEFtoMapTransform();

    ros::ServiceServer maneuver_srv_ = nh.advertiseService("plugins/PlatooningStrategicPlugin/plan_maneuvers",
                                            &PlatoonStrategicPlugin::plan_maneuver_cb, &worker);
    ros::Subscriber mob_request_sub = nh.subscribe("incoming_mobility_request", 1, &PlatoonStrategicPlugin::mob_req_cb,  &worker);
    ros::Subscriber mob_response_sub = nh.subscribe("incoming_mobility_response", 1, &PlatoonStrategicPlugin::mob_resp_cb,  &worker);
    ros::Subscriber mob_operation_sub = nh.subscribe("incoming_mobility_operation", 1, &PlatoonStrategicPlugin::mob_op_cb,  &worker);
    ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 1, &PlatoonStrategicPlugin::pose_cb,  &worker);
    ros::Subscriber current_twist_sub = nh.subscribe("current_velocity", 1, &PlatoonStrategicPlugin::twist_cb,  &worker);
    ros::Subscriber bsm_sub = nh.subscribe("bsm_outbound", 1, &PlatoonStrategicPlugin::bsm_cb,  &worker);
    ros::Subscriber cmd_sub = nh.subscribe("twist_raw", 1, &PlatoonStrategicPlugin::cmd_cb,  &worker);
    ros::Subscriber georeference_sub = nh.subscribe("georeference", 1, &PlatoonStrategicPlugin::georeference_cb, &worker);

    
    ros::Timer discovery_pub_timer_ = nh.createTimer(
            ros::Duration(ros::Rate(10.0)),
            [&worker](const auto&) { worker.onSpin(); });

    ros::CARMANodeHandle::spin();
  }
};

}  // namespace platoon_strategic