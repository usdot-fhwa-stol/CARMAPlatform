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

#include "entry_manager.h"
#include <gtest/gtest.h>
#include <iostream>

namespace health_monitor
{
    
    TEST(EntryManagerTest, testAddNewEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(true, res.begin()->available_);
        EXPECT_EQ(false, res.begin()->active_);
        EXPECT_EQ("cruising_plugin", res.begin()->name_);
        EXPECT_EQ(1000, res.begin()->timestamp_);
        EXPECT_EQ(1, res.begin()->type_);
    }

    TEST(EntryManagerTest, testUpdateEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "cruising_plugin", 2000, 3, "");
        em.update_entry(new_entry);
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(true, res.begin()->active_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("cruising_plugin", res.begin()->name_);
        EXPECT_EQ(1, res.begin()->type_);
    }

    TEST(EntryManagerTest, testDeleteEntry)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "cruising_plugin_2", 2000, 3, "");
        em.update_entry(new_entry);
        em.delete_entry("cruising_plugin");
        std::vector<Entry> res = em.get_entries();
        EXPECT_EQ(1, res.size());
        EXPECT_EQ(false, res.begin()->available_);
        EXPECT_EQ(true, res.begin()->active_);
        EXPECT_EQ(2000, res.begin()->timestamp_);
        // the following two attributes should not change once set
        EXPECT_EQ("cruising_plugin_2", res.begin()->name_);
        EXPECT_EQ(3, res.begin()->type_);
    }

    TEST(EntryManagerTest, testGetEntryByValidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        boost::optional<Entry> res = em.get_entry_by_name("cruising_plugin");
        EXPECT_EQ(true, res->available_);
        EXPECT_EQ(false, res->active_);
        EXPECT_EQ(1000, res->timestamp_);
        EXPECT_EQ(1, res->type_);
        EXPECT_EQ("cruising_plugin", res->name_);
    }

    TEST(EntryManagerTest, testGetEntryByInvalidName)
    {
        EntryManager em;
        // params: bool available, bool active, std::string name, long timestamp, uint8_t type
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        boost::optional<Entry> res = em.get_entry_by_name("plugin");
        if(!res)
        {
            ASSERT_TRUE(1 == 1);
        } else
        {
            ASSERT_TRUE(1 == 2);
        }
    }

    TEST(EntryManagerTest, testRequiredEntryCheck)
    {
        std::vector<std::string> required_names;
        required_names.push_back("cruising");
        required_names.push_back("cruising_plugin");
        EntryManager em(required_names);
        Entry entry(true, false, "cruising_plugin", 1000, 1, "");
        em.update_entry(entry);
        Entry new_entry(false, true, "cruising_plugin_2", 2000, 3, "");
        em.update_entry(new_entry);
        EXPECT_EQ(true, em.is_entry_required("cruising_plugin"));
        EXPECT_EQ(true, em.is_entry_required("cruising"));
        EXPECT_EQ(false, em.is_entry_required("cruising_plugin_2"));
    }

        TEST(EntryManagerTest, testTruckLidarGpsEntryCheck)
    {   
        std::vector<std::string> required_entries;
        required_entries.push_back("ssc");
 
        std::vector<std::string> lidar_gps_entries;
        lidar_gps_entries.push_back("lidar1");
        lidar_gps_entries.push_back("lidar2");
        lidar_gps_entries.push_back("gps");

        EntryManager em(required_entries,lidar_gps_entries);

        EXPECT_EQ(0, em.is_lidar_gps_entry_required("lidar1"));
        EXPECT_EQ(1, em.is_lidar_gps_entry_required("lidar2"));
        EXPECT_EQ(2, em.is_lidar_gps_entry_required("gps"));
    }

        TEST(EntryManagerTest, testCarLidarGpsEntryCheck)
    {   
        std::vector<std::string> required_entries;
        required_entries.push_back("ssc");
 
        std::vector<std::string> lidar_gps_entries;
        lidar_gps_entries.push_back("lidar");
        lidar_gps_entries.push_back("gps");

        EntryManager em(required_entries,lidar_gps_entries);

        EXPECT_EQ(0, em.is_lidar_gps_entry_required("lidar"));
        EXPECT_EQ(1, em.is_lidar_gps_entry_required("gps"));
    }
    
}
