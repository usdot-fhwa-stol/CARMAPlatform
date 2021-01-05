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

#include "route_state_worker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RouteStateTest, testStateTransitions)
{
    route::RouteStateWorker worker;
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_ABORTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_LOADED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::SELECTION, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_STARTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::FOLLOWING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_DEPARTED);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_LOADED);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::SELECTION, worker.get_route_state());
}
