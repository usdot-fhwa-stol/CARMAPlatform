#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
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

cd "$(dirname "$0")"/../carma

MAJOR_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int major" | grep -o "[0-9]*")
INTERMEDIATE_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int intermediate" | grep -o "[0-9]*")
MINOR_RELEASE=$(echo "$GUIDANCE_VERSION" | grep "int minor" | grep -o "[0-9]*")
BUILD_NUMBER=`echo $VER_STRING | awk '{ print $1 }'`
BUILD_SUFFIX=`echo $VER_STRING | awk '{ print $2 }'`
FULL_VERSION_STRING="$MAJOR_RELEASE.$INTERMEDIATE_RELEASE.$MINOR_RELEASE.$BUILD_NUMBER-$BUILD_SUFFIX"
FULL_VERSION_STRING="$(echo $FULL_VERSION_STRING | sed 's/(.*)\(.*\)/\1/')"
echo "$FULL_VERSION_STRING"
