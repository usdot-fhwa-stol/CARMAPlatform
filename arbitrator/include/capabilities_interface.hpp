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

#ifndef __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_HPP__
#define __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_HPP__

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <vector>
#include <map>
#include <unordered_set>
#include <string>
#include <cav_srvs/PluginList.h>
#include <cav_srvs/GetPluginApi.h>

namespace arbitrator
{
    /**
     * \brief Generic interface for interacting with Plugins via their capabilities
     *      instead of directly by their topics.
     */
    class CapabilitiesInterface
    {
        public:
            /**
             * \brief Constructor for Capabilities interface
             * \param nh A publically addressesed ("/") ros::NodeHandle
             */
            CapabilitiesInterface(ros::NodeHandle *nh): nh_(nh) {
                sc_s = nh_->serviceClient<cav_srvs::GetPluginApi>("plugins/get_strategic_plugin_by_capability");
            };

            /**
             * \brief Initialize the Capabilities interface by querying the Health Monitor
             *      node and processing the plugins that are returned.
             */
            // void initialize();

            /**
             * \brief Get the list of topics that respond to the capability specified by
             *      the query string
             * 
             * \param query_string The string name of the capability to look for
             * \return A list of all responding topics, if any are found.
             */
            std::vector<std::string> get_topics_for_capability(const std::string& query_string);


            /**
             * \brief Template function for calling all nodes which respond to a service associated
             *      with a particular capabilitiy. Will send the service request to all nodes and 
             *      aggregate the responses.
             * 
             * \tparam MSrv The typename of the service message
             * \param query_string The string name of the capability to look for
             * \param The message itself to send
             * \return A map matching the topic name that responded -> the response
             */
            template<typename MSrv>
            std::map<std::string, MSrv> multiplex_service_call_for_capability(std::string query_string, MSrv msg);

            const static std::string STRATEGIC_PLAN_CAPABILITY;
        protected:
        private:
            ros::NodeHandle *nh_;

            ros::ServiceClient sc_s;
            std::unordered_set <std::string> capabilities_ ; 


            
    };
};

#include "capabilities_interface.tpp"

#endif
