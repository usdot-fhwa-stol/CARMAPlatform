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

#include "entry.h"

namespace health_monitor
{
    Entry::Entry(bool available, bool active, const std::string& name, long timestamp, uint8_t type, const std::string& capability) :
        available_(available), active_(active), name_(name), timestamp_(timestamp), type_(type), capability_(capability) {}

}