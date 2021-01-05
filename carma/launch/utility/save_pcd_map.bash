#!/bin/bash

#  Copyright (C) 2020-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

set -e

PCD_FILE="$1"

if [ -z "$PCD_FILE" ]; then
    FILENAME="$(date '+%Y-%m-%d_%H-%M-%S')_generated_map.pcd"
    PCD_FILE="/tmp/$FILENAME"
    echo "No file save path provided. File will be saved to:"
    echo "$PCD_FILE"
fi

rostopic pub -l /config/ndt_mapping_output autoware_config_msgs/ConfigNDTMappingOutput \
"header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
filename: '$PCD_FILE'
filter_res: 0.2"
