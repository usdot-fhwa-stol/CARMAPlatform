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
#include <functional>
#include <ros/time.h>
#include <mutex>
#include <memory>
#include <unordered_map>
#include <carma_wm_ctrl/Geofence.h>
#include <carma_utils/timers/Timer.h>
#include <carma_utils/timers/TimerFactory.h>

namespace carma_wm_ctrl
{
/**
 * @brief A GeofenceScheduler is responsable for notifying the user when a geofence is active or inactive according to
 * its schedule
 */
class GeofenceScheduler
{
  using Timer = carma_utils::timers::Timer;
  using TimerFactory = carma_utils::timers::TimerFactory;
  using TimerPtr = std::unique_ptr<Timer>;

  std::mutex mutex_;
  std::unique_ptr<TimerFactory> timerFactory_;
  std::unordered_map<uint32_t, std::pair<TimerPtr, bool>> timers_;  // Pairing of timers with their Id and valid status
  std::unique_ptr<Timer> deletion_timer_;
  std::function<void(std::shared_ptr<Geofence>)> active_callback_;
  std::function<void(std::shared_ptr<Geofence>)> inactive_callback_;
  uint32_t next_id_ = 0;  // Timer id counter

public:
  /**
   * @brief Constructor which takes in a TimerFactory. Timers from this factory will be used to generate the triggers
   * for goefence activity.
   *
   * @param timerFactory A pointer to a TimerFactory which can be used to generate timers for geofence triggers.
   */
  GeofenceScheduler(std::unique_ptr<TimerFactory> timerFactory);

  /**
   * @brief Add a geofence to the scheduler. This will cause it to trigger an event when it becomes active or goes
   * inactive according to its schedule
   *
   * @param geofence The geofence to be added
   */
  void addGeofence(std::shared_ptr<Geofence> gf_ptr);

  /**
   * @brief Method which allows the user to set a callback which will be triggered when a geofence becomes active
   *
   * @param active_callback The callback which will be triggered
   */
  void onGeofenceActive(std::function<void(std::shared_ptr<Geofence>)> active_callback);
  /**
   * @brief Method which allows the user to set a callback which will be triggered when a geofence becomes in-active
   *
   * @param inactive_callback The callback which will be triggered
   */
  void onGeofenceInactive(std::function<void(std::shared_ptr<Geofence>)> inactive_callback);

  /**
   * @brief Clears the expired timers from the memory of this scheduler
   */
  void clearTimers();

private:
  /**
   * @brief Generates the next id to be used for a timer
   *
   * @return The next available timer id
   */
  uint32_t nextId();

  /**
   * @brief The callback which is triggered when a geofence becomes active
   *        This will call the user set active_callback set from the onGeofenceActive function
   *
   * @param event The record of the timer event causing this to trigger
   * @param gf The geofence which is being activated
   * @param schedule_id index number of the schedule being used corresponding to this geofence
   * @param timer_id The id of the timer which caused this callback to occur
   */
  void startGeofenceCallback(const ros::TimerEvent& event, std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id);
  /**
   * @brief The callback which is triggered when a geofence becomes in-active
   *        This will call the user set inactive_callback set from the onGeofenceInactive function
   *
   * @param event The record of the timer event causing this to trigger
   * @param gf The geofence which is being un-activated
   * @param schedule_id index number of the schedule being used corresponding to this geofence
   * @param timer_id The id of the timer which caused this callback to occur
   */
  void endGeofenceCallback(const ros::TimerEvent& event, std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id);
};
}  // namespace carma_wm_ctrl