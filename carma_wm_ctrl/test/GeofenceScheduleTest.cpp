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

using ::testing::_;
using ::testing::A;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;
using ::testing::ReturnArg;

namespace carma_wm_ctrl
{
TEST(GeofenceSchedule, scheduleStarted)
{
  GeofenceSchedule sch;
  sch.schedule_start_ = ros::Time(0);
  sch.schedule_end_ = ros::Time(1);
  sch.control_start_ = ros::Duration(0);
  sch.control_duration_ = ros::Duration(1);
  sch.control_offset_ = ros::Duration(0);
  sch.control_span_ = ros::Duration(1);
  sch.control_period_ = ros::Duration(2);

  ASSERT_TRUE(sch.scheduleStarted(ros::Time(0)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(0.9)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1.0)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1.1)));

  sch.schedule_start_ = ros::Time(1579882740.000);  // EST Mon Jan 24 1970 11:19:00
  sch.schedule_end_ = ros::Time(1579886340.000);    // 1 hr total duration

  ASSERT_FALSE(sch.scheduleStarted(ros::Time(1579882739.000)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1579882740.000)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1579882741.000)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1579886340.000)));
  ASSERT_TRUE(sch.scheduleStarted(ros::Time(1579886341.000)));
}

TEST(GeofenceSchedule, getNextInterval)
{
  // Test before start

  GeofenceSchedule sch(ros::Time(1), ros::Time(6), ros::Duration(2), ros::Duration(1), ros::Duration(0), ros::Duration(1),
                       ros::Duration(2)  // This means the next schedule is a 4 (2+2)
  );

  // Test before control start
  ASSERT_NEAR(2.0, sch.getNextInterval(ros::Time(0)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(0)).first);
  // Test after start but before control_start
  ASSERT_NEAR(2.0, sch.getNextInterval(ros::Time(1.5)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(1.5)).first);
  // Test between first control_start and control_end
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(2.5)).second.toSec(), 0.00001);
  ASSERT_TRUE(sch.getNextInterval(ros::Time(2.5)).first);
  // Test after control ends
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(3.5)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(3.5)).first);

  sch = GeofenceSchedule(ros::Time(1), ros::Time(6), ros::Duration(2), ros::Duration(3), ros::Duration(0), ros::Duration(1),
                         ros::Duration(2)  // This means the next schedule is a 4 (2+2)
  );
  // Test between end of first control and start of second
  ASSERT_NEAR(4.0, sch.getNextInterval(ros::Time(3.5)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(3.5)).first);
  // Test between 2nd control start and control end
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(4.5)).second.toSec(), 0.00001);
  ASSERT_TRUE(sch.getNextInterval(ros::Time(4.5)).first);
  // Test after control_end
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(5.5)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(5.5)).first);
  // Test other day of the week
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(90000)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(90000)).first);
  // Test after schedule end
  ASSERT_NEAR(0.0, sch.getNextInterval(ros::Time(7.0)).second.toSec(), 0.00001);
  ASSERT_FALSE(sch.getNextInterval(ros::Time(7.0)).first);
}
}  // namespace carma_wm_ctrl