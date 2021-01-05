#pragma once
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
#include <ros/time.h>
#include <unordered_set>
#include <boost/date_time/gregorian/greg_weekday.hpp>
#include <boost/date_time/gregorian/greg_date.hpp>

namespace carma_wm_ctrl
{
class GeofenceSchedule
{
public:
  ros::Time schedule_start_;  // Overall schedule start
  ros::Time schedule_end_;    // Overall schedule end

  ros::Duration control_start_;     // Duration from start of day
  ros::Duration control_duration_;  // Duration from start of control_start
  ros::Duration control_offset_;    // Duration from start of day (specifies start time for repeat parameters - span, period)
  ros::Duration control_span_;      // Duration of active status. Starts from offset
  ros::Duration control_period_;    // Interval between active status within control_duration since control_start

  using DayOfTheWeekSet =
      std::unordered_set<boost::gregorian::greg_weekday, std::hash<int>>;  // Set of week days where geofence is active.
                                                                           // Uses int hashing function
  DayOfTheWeekSet week_day_set_;  // NOTE: If no day of the week is included then all should be

  /**
   * @brief Default Constructor does not initialize any members
   */
  GeofenceSchedule();

  /**
   * @brief Constructor
   *
   * @param schedule_start    Overall schedule start in UTC
   * @param schedule_end      Overall schedule end in UTC
   * @param control_start     Duration from start of day
   * @param control_duration  Duration from start of control_start
   * @param control_offset    Duration from start of day (specifies start time for repeat parameters - span, period)
   * @param control_span      Duration of active status. Starts from offset
   * @param control_period    Interval between active status within control_duration since control_start
   * @param week_day_set      Set of days of the week which this schedule applies to. Defaults as all days of the week
   */
  GeofenceSchedule(ros::Time schedule_start, ros::Time schedule_end, ros::Duration control_start,
                   ros::Duration control_duration, ros::Duration control_offset, ros::Duration control_span, ros::Duration control_period,
                   DayOfTheWeekSet week_day_set = { 0, 1, 2, 3, 4, 5, 6 });  // Include all weekdays as default (0-6) ->
                                                                             // (Sun-Sat)

  /**
   * @brief Returns true if the schedule has expired by the provided time
   *
   * @param time UTC time to compare
   *
   * @return True if time > schedule_end
   */
  bool scheduleExpired(const ros::Time& time = ros::Time::now()) const;

  /**
   * @brief Returns true if the schedule has started by the provided time
   *
   * @param time UTC time to compare
   *
   * @return True if time > schedule_start
   */
  bool scheduleStarted(const ros::Time& time = ros::Time::now()) const;

  /**
   * @brief Returns the start time of the next active interval defined by this schedule
   *
   * @param time UTC time to compare
   *
   * @return Returns a pair with the following semantics
   *          First Element: A boolean indicating if the provided time is within a currently active control period
   *          Second Element: The start time of the next scheduled active control interval within the current day.
   * Returns ros::Time(0) when the schedule is expired or the next interval will be on a different day of the week or
   * after schedule end
   *
   * TODO the UTC offset is provided in the geofence spec but for now we will ignore and assume all times are UTC
   */
  std::pair<bool, ros::Time> getNextInterval(const ros::Time& time) const;
};
}  // namespace carma_wm_ctrl