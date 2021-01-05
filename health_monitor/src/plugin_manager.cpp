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

#include "plugin_manager.h"

namespace health_monitor
{

    PluginManager::PluginManager(const std::vector<std::string>& require_plugin_names,
                                 const std::string& service_prefix,
                                 const std::string& strategic_service_suffix,
                                 const std::string& tactical_service_suffix)
                                 : service_prefix_(service_prefix),
                                   strategic_service_suffix_(strategic_service_suffix),
                                   tactical_service_suffix_(tactical_service_suffix),
                                   em_(EntryManager(require_plugin_names))
    {}

    void PluginManager::get_registered_plugins(cav_srvs::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(auto i = plugins.begin(); i < plugins.end(); ++i)
        {
            cav_msgs::Plugin plugin;
            plugin.activated = i->active_;
            plugin.available = i->available_;
            plugin.name = i->name_;
            plugin.type = i->type_;
            res.plugins.push_back(plugin);
        }
    }

    void PluginManager::get_active_plugins(cav_srvs::PluginListResponse& res)
    {
        std::vector<Entry> plugins = em_.get_entries();
        // convert to plugin list
        for(auto i = plugins.begin(); i < plugins.end(); ++i)
        {
            if(i->active_)
            {
                cav_msgs::Plugin plugin;
                plugin.activated = true;
                plugin.available = i->available_;
                plugin.name = i->name_;
                plugin.type = i->type_;
                res.plugins.push_back(plugin);
            }
        }
    }

    bool PluginManager::activate_plugin(const std::string& name, const bool activate)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(name);
        if(requested_plugin)
        {
            // params: bool available, bool active, std::string name, long timestamp, uint8_t type
            Entry updated_entry(requested_plugin->available_, activate, requested_plugin->name_, 0, requested_plugin->type_, requested_plugin->capability_);
            em_.update_entry(updated_entry);
            return true;
        }
        return false;
    }

    void PluginManager::update_plugin_status(const cav_msgs::PluginConstPtr& msg)
    {
        boost::optional<Entry> requested_plugin = em_.get_entry_by_name(msg->name);
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry plugin(msg->available, msg->activated, msg->name, 0, msg->type, msg->capability);
        // if it already exists, we do not change its activation status
        if(requested_plugin)
        {
            plugin.active_ = requested_plugin->active_;
        } else if(em_.is_entry_required(msg->name))
        {
            plugin.active_ = true;
        }
        em_.update_entry(plugin);
    }

    bool PluginManager::get_tactical_plugins_by_capability(cav_srvs::GetPluginApiRequest& req, cav_srvs::GetPluginApiResponse& res)
    {
        for(const auto& plugin : em_.get_entries())
        {
            if(plugin.type_ == cav_msgs::Plugin::TACTICAL &&
               (req.capability.size() == 0 || (plugin.capability_.compare(0, req.capability.size(), req.capability) == 0 && plugin.active_ && plugin.available_)))
            {
                res.plan_service.push_back(service_prefix_ + plugin.name_ + tactical_service_suffix_);
            }
        }
        return true;
    }

    bool PluginManager::get_strategic_plugins_by_capability(cav_srvs::GetPluginApiRequest& req, cav_srvs::GetPluginApiResponse& res)
    {
        for(const auto& plugin : em_.get_entries())
        {
            if(plugin.type_ == cav_msgs::Plugin::STRATEGIC && 
                (req.capability.size() == 0 || (plugin.capability_.compare(0, req.capability.size(), req.capability) == 0 && plugin.active_ && plugin.available_)))
            {
                res.plan_service.push_back(service_prefix_ + plugin.name_ + strategic_service_suffix_);
            }
        }
        return true;
    }

}
