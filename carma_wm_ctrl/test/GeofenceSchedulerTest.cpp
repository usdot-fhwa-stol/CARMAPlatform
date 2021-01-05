/*
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
 */

#include <gmock/gmock.h>
#include <carma_wm_ctrl/GeofenceSchedule.h>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_wm_ctrl/GeofenceScheduler.h>
#include <memory>
#include <chrono>
#include <ctime>
#include <atomic>
#include <carma_utils/testing/TestHelpers.h>
#include <carma_utils/timers/testing/TestTimer.h>
#include <carma_utils/timers/testing/TestTimerFactory.h>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

using carma_utils::timers::testing::TestTimer;
using carma_utils::timers::testing::TestTimerFactory;

namespace carma_wm_ctrl
{
TEST(GeofenceScheduler, Constructor)
{
  GeofenceScheduler scheduler(std::make_unique<TestTimerFactory>());  // Create scheduler with test timers. Having this
                                                                      // check helps verify that the timers do not crash
                                                                      // on destruction
}

TEST(GeofenceScheduler, addGeofence)
{
  // Test adding then evaulate if the calls to active and inactive are done correctly
  // Finally test cleaing the timers
  auto gf_ptr = std::make_shared<Geofence>(Geofence());
  
  boost::uuids::uuid first_id = boost::uuids::random_generator()(); 
  std::size_t first_id_hashed = boost::hash<boost::uuids::uuid>()(first_id);
  gf_ptr->id_ = first_id;

  gf_ptr->schedules.push_back(
      GeofenceSchedule(ros::Time(1),  // Schedule between 1 and 8
                       ros::Time(8),
                       ros::Duration(2),    // Starts at 2
                       ros::Duration(3.5),  // Ends at by 5.5
                       ros::Duration(0),    // repetition start 0 offset, so still start at 2
                       ros::Duration(1),    // Duration of 1 and interval of 2 so active durations are (2-3 and 4-5)
                       ros::Duration(2)));
  ros::Time::setNow(ros::Time(0));  // Set current time

  GeofenceScheduler scheduler(std::make_unique<TestTimerFactory>());  // Create scheduler
  std::atomic<uint32_t> active_call_count(0);
  std::atomic<uint32_t> inactive_call_count(0);
  std::atomic<std::size_t> last_active_gf(0);
  std::atomic<std::size_t> last_inactive_gf(0);
  scheduler.onGeofenceActive([&](std::shared_ptr<Geofence> gf_ptr) {
    active_call_count.store(active_call_count.load() + 1);
    // atomic is not working for boost::uuids::uuid, so hash it
    last_active_gf.store(boost::hash<boost::uuids::uuid>()(gf_ptr->id_));
  });

  scheduler.onGeofenceInactive([&](std::shared_ptr<Geofence> gf_ptr) {
    inactive_call_count.store(inactive_call_count.load() + 1);
    // atomic is not working for boost::uuids::uuid, so hash it
    last_inactive_gf.store(boost::hash<boost::uuids::uuid>()(gf_ptr->id_));
  });

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  scheduler.addGeofence(gf_ptr);

  ros::Time::setNow(ros::Time(1.0));  // Set current time

  ASSERT_EQ(0, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_active_gf.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(2.1));  // Set current time

  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, first_id_hashed, last_active_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(0, inactive_call_count.load());
  ASSERT_EQ(0, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(3.1));  // Set current time

  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, first_id_hashed, last_inactive_gf));
  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  ros::Time::setNow(ros::Time(3.5));  // Set current time

  ASSERT_EQ(1, active_call_count.load());
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());

  ros::Time::setNow(ros::Time(4.2));  // Set current time

  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 2, active_call_count));
  ASSERT_EQ(1, inactive_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  ros::Time::setNow(ros::Time(5.5));  // Set current time

  ASSERT_TRUE(carma_utils::testing::waitForEqOrTimeout(10.0, 2, inactive_call_count));
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());

  ros::Time::setNow(ros::Time(9.5));  // Set current time

  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());

  // Basic check that expired geofence is not added
  boost::uuids::uuid second_id = boost::uuids::random_generator()();
  gf_ptr->id_ = second_id;
  scheduler.addGeofence(gf_ptr);

  ros::Time::setNow(ros::Time(11.0));  // Set current time

  carma_utils::testing::waitForEqOrTimeout(3.0, 10, inactive_call_count);  // Let some time pass just in case
  ASSERT_EQ(2, inactive_call_count.load());
  ASSERT_EQ(2, active_call_count.load());
  ASSERT_EQ(first_id_hashed, last_active_gf.load());
  ASSERT_EQ(first_id_hashed, last_inactive_gf.load());
}

}  // namespace carma_wm_ctrl