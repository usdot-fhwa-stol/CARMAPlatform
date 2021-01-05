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

#include <carma_wm_ctrl/GeofenceScheduler.h>

namespace carma_wm_ctrl
{
using std::placeholders::_1;

GeofenceScheduler::GeofenceScheduler(std::unique_ptr<TimerFactory> timerFactory)
  : timerFactory_(std::move(timerFactory))
{
  // Create repeating loop to clear geofence timers which are no longer needed
  deletion_timer_ =
      timerFactory_->buildTimer(nextId(), ros::Duration(1), std::bind(&GeofenceScheduler::clearTimers, this));
}

uint32_t GeofenceScheduler::nextId()
{
  next_id_++;
  return next_id_;
}

void GeofenceScheduler::clearTimers()
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto it = timers_.begin();

  // Erase all expired timers_ using iterators to ensure operation is safe
  while (it != timers_.end())
  {
    // Check if timer is marked for deletion
    if (it->second.second)
    {
      // erase() function returns the iterator of the next
      // to last deleted element.
      it = timers_.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void GeofenceScheduler::addGeofence(std::shared_ptr<Geofence> gf_ptr)
{
  std::lock_guard<std::mutex> guard(mutex_);

  ROS_INFO_STREAM("Attempting to add Geofence with Id: " << gf_ptr->id_);

  // Create timer for next start time
  for (auto schedule_idx = 0; schedule_idx < gf_ptr->schedules.size(); schedule_idx++)
  {
    auto interval_info = gf_ptr->schedules[schedule_idx].getNextInterval(ros::Time::now());
    ros::Time startTime = interval_info.second;
    if (!interval_info.first && startTime == ros::Time(0))
    {
      ROS_WARN_STREAM(
          "Failed to add geofence as its schedule did not contain an active or upcoming control period. GF Id: "
          << gf_ptr->id_);
      return;
    }
    // If this geofence is currently active set the start time to now
    if (interval_info.first)
    {
      startTime = ros::Time::now();
    }

    int32_t timer_id = nextId();

    // Build timer to trigger when this geofence becomes active
    TimerPtr timer = timerFactory_->buildTimer(
        timer_id, startTime - ros::Time::now(),
        std::bind(&GeofenceScheduler::startGeofenceCallback, this, _1, gf_ptr, schedule_idx, timer_id), true, true);

    timers_[timer_id] = std::make_pair(std::move(timer), false);  // Add start timer to map by Id
  }
}

void GeofenceScheduler::startGeofenceCallback(const ros::TimerEvent& event, std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  ros::Time endTime = ros::Time::now() + gf_ptr->schedules[schedule_id].control_span_;

  ROS_INFO_STREAM("Activating Geofence with Id: " << gf_ptr->id_);

  active_callback_(gf_ptr);

  // Build timer to trigger when this geofence becomes inactive
  int32_t ending_timer_id = nextId();
  TimerPtr timer = timerFactory_->buildTimer(
      ending_timer_id, endTime - ros::Time::now(),
      std::bind(&GeofenceScheduler::endGeofenceCallback, this, _1, gf_ptr, schedule_id, ending_timer_id), true, true);
  timers_[ending_timer_id] = std::make_pair(std::move(timer), false);  // Add end timer to map by Id

  timers_[timer_id].second = true;  // Mark start timer for deletion
}

void GeofenceScheduler::endGeofenceCallback(const ros::TimerEvent& event, std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  ROS_INFO_STREAM("Deactivating Geofence with Id: " << gf_ptr->id_);

  inactive_callback_(gf_ptr);
  timers_[timer_id].second = true;  // Mark timer for deletion

  // Determine if a new timer is needed for this geofence
  auto interval_info = gf_ptr->schedules[schedule_id].getNextInterval(ros::Time::now());
  ros::Time startTime = interval_info.second;

  // If this geofence should currently be active set the start time to now
  if (interval_info.first)
  {
    startTime = ros::Time::now();
  }

  if (!interval_info.first && startTime == ros::Time(0))
  {
    // No more active periods for this geofence so return
    return;
  }

  // Build timer to trigger when this geofence becomes active
  int32_t start_timer_id = nextId();

  TimerPtr timer = timerFactory_->buildTimer(
      start_timer_id, startTime - ros::Time::now(),
      std::bind(&GeofenceScheduler::startGeofenceCallback, this, _1, gf_ptr, schedule_id, start_timer_id), true, true);

  timers_[start_timer_id] = std::make_pair(std::move(timer), false);  // Add start timer to map by Id
}

void GeofenceScheduler::onGeofenceActive(std::function<void(std::shared_ptr<Geofence>)> active_callback)
{
  std::lock_guard<std::mutex> guard(mutex_);
  active_callback_ = active_callback;
}

void GeofenceScheduler::onGeofenceInactive(std::function<void(std::shared_ptr<Geofence>)> inactive_callback)
{
  std::lock_guard<std::mutex> guard(mutex_);
  inactive_callback_ = inactive_callback;
}
}  // namespace carma_wm_ctrl