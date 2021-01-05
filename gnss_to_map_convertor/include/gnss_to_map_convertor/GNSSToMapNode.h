#pragma once

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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <cav_srvs/GetTransform.h>
#include <wgs84_utils/wgs84_utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps_msgs/NovatelDualAntennaHeading.h>
#include <novatel_gps_msgs/NovatelPosition.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <gnss_to_map_convertor/GNSSToMapConvertor.h>
#include <carma_utils/CARMAUtils.h>

namespace gnss_to_map_convertor {
  /**
   * \class GNSSToMapNode
   * \brief ROS Node which maintains a tf2 transform tree which can be accessed by other nodes.
   *
   * The get_transform service can be used to obtain coordinate transformations between two frames.
   * Only transforms published on the /tf or /tf_static topics are recorded by this node.
   */
  class GNSSToMapNode {

    private:
      // Buffer which holds the tree of transforms
      tf2_ros::Buffer tfBuffer_;
      // tf2 listeners. Subscribes to the /tf and /tf_static topics
      tf2_ros::TransformListener tfListener_ {tfBuffer_};
      
      // Ros node handle
      ros::CARMANodeHandle cnh_;
      ros::CARMANodeHandle p_cnh_ {"~"}; // Private node handle initialized in constructor

      ros::Publisher ecef_pose_pub_;
      ros::Publisher map_pose_pub_;

      ros::Subscriber fix_sub_;

      tf2::Transform baselink_in_sensor_; 
      tf2::Transform sensor_in_ned_; 
      bool baselink_in_sensor_set_ = false;
      std::string base_link_frame_ = "base_link";
      std::string earth_frame_ = "earth";
      std::string map_frame_ = "map";
      std::string ned_heading_frame_ = "ned_heading";

      /**
       * @brief GPSFix callback which publishes the updated ecef and map poses
       * 
       * @param fix_msg The gnss fix message which must contain the position and heading information
       */ 
      void fixCb(const gps_common::GPSFixConstPtr& fix_msg);

    public:
      /**
       * @brief Default Constructor
       */
      GNSSToMapNode();

      /**
       * @brief Starts the Node
       *
       * @return 0 on exit with no errors
       */
      int run();
  };
}