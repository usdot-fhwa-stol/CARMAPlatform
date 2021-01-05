#!/bin/bash

# Copyright (C) 2018-2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.


PARAMS_DIR=$(find .. -path "*carma/launch/params" -type d)
if [[ -z $PARAMS_DIR ]]; then
    echo "Unable to find params in local dir, looking in deployment dir"
    $PARAMS_DIR=$(find /opt/carma -path "*launch/params" -type d)
    if [[ -z $PARAMS_DIR ]]; then
        echo "Unable to locate params dir in deployment!"
        exit 1
    fi
else
    echo "Configuration discovered in $PARAMS_DIR"
fi

GUIDANCE_PARAMS=$PARAMS_DIR/GuidanceParams.yaml
MESSAGE_PARAMS=$PARAMS_DIR/MessageParams.yaml

DEFAULT_DESIRED_ACC_TIMEGAP="2.0"
DEFAULT_MIN_ACC_STANDOFF_DISTANCE="5.0"
DEFAULT_MAX_DOWNTRACK_ERROR="100.0"
DEFAULT_MAX_SPEED_ERROR="100.0"

get_acc() {
    TIMEGAP_SET=$(grep 'desired_acc_timegap: 0.0' $GUIDANCE_PARAMS)
    STANDOFF_SET=$(grep 'min_acc_standoff_distance: 0.0' $GUIDANCE_PARAMS)

    if [[ -z "$TIMEGAP_SET$STANDOFF_SET" ]]; then
        echo "acc_enabled: true"
    else
        echo "acc_enabled: false"
    fi
}

set_acc() {
    echo "Setting acc_enabled status to $1 in $GUIDANCE_PARAMS"
    if [[ "$1" == "true" ]]
    then
        sed -i.bak "s/desired_acc_timegap:.*/desired_acc_timegap: $DEFAULT_DESIRED_ACC_TIMEGAP/;
        s/min_acc_standoff_distance:.*/min_acc_standoff_distance: $DEFAULT_MIN_ACC_STANDOFF_DISTANCE/" \
        $GUIDANCE_PARAMS
    else
        sed -i.bak "s/desired_acc_timegap:.*/desired_acc_timegap: 0.0/;
        s/min_acc_standoff_distance:.*/min_acc_standoff_distance: 0.0/" \
        $GUIDANCE_PARAMS
    fi

    diff $GUIDANCE_PARAMS.bak $GUIDANCE_PARAMS
}

set_bsm() {
    echo "Setting bsm_enabled status to $1 in $MESSAGE_PARAMS"
    if [[ "$1" == "true" ]]
    then
        sed -i.bak "s/publish_outbound_bsm:.*/publish_outbound_bsm: true/" $MESSAGE_PARAMS
    else
        sed -i.bak "s/publish_outbound_bsm:.*/publish_outbound_bsm: false/" $MESSAGE_PARAMS
    fi

    diff $MESSAGE_PARAMS.bak $MESSAGE_PARAMS
}

get_bsm() {
    if [[ $(grep "publish_outbound_bsm: true" $MESSAGE_PARAMS) ]]; then
        echo "bsm_enabled: true"
    else
        echo "bsm_enabled: false"
    fi
}

set_tracking() {
    echo "Setting tracking_enabled to $1 in $GUIDANCE_PARAMS"
    if [[ "$1" == "true" ]]
    then
        sed -i.bak "s/max_downtrack_error:.*/max_downtrack_error: $DEFAULT_MAX_DOWNTRACK_ERROR/;
        s/max_speed_error:.*/max_speed_error: $DEFAULT_MAX_SPEED_ERROR/" \
        $GUIDANCE_PARAMS
    else
        sed -i.bak 's/max_downtrack_error:.*/max_downtrack_error: 999999999.0/;
        s/max_speed_error:.*/max_speed_error: 999999999.0/' \
        $GUIDANCE_PARAMS
    fi

    diff $GUIDANCE_PARAMS.bak $GUIDANCE_PARAMS
}

get_tracking() {
    MAX_DOWNTRACK_ERROR_SET=$(grep 'max_downtrack_error: 999999999.0' $GUIDANCE_PARAMS)
    MAX_SPEED_ERROR_SET=$(grep 'max_speed_error: 999999999.0' $GUIDANCE_PARAMS)

    if [[ -z "$MAX_DOWNTRACK_ERROR_SET$MAX_SPEED_ERROR_SET" ]]; then
        echo "tracking_enabled: true"
    else
        echo "tracking_enabled: false"
    fi
}

reset_cfg() {
    BRANCH_NAME=$(git branch | sed -n '/\* /s///p')
    echo "Checking out $GUIDANCE_PARAMS from git branch $BRANCH_NAME..."
    mv $GUIDANCE_PARAMS $GUIDANCE_PARAMS.bak
    git checkout $GUIDANCE_PARAMS

    diff $GUIDANCE_PARAMS.bak $GUIDANCE_PARAMS

    echo "Checking out $MESSAGE_PARAMS from git branch $BRANCH_NAME..."
    mv $MESSAGE_PARAMS $MESSAGE_PARAMS.bak
    git checkout $MESSAGE_PARAMS
}

print_help() {
    echo "configure.sh usage:"
    echo "configure.sh {summarize | acc_enabled (true/false) | tracking_enabled (true/false) | bsm_enabled (true/false) | reset | help}"
    echo "-- Configurations will be modified in place, backup files will be saved in between commands as *.bak"
    echo "-- Commands may be run in sequence, e.g.: configure.sh acc_enabled true tracking_enabled false"
}

# Main arg parsing loop
argc=$#
argv=($@)
for (( j=0; j<argc; j++ )); do
    case ${argv[j]} in
        summarize)
        get_acc
        get_tracking
        get_bsm
        ;;
        acc_enabled)
        set_acc ${argv[++j]}
        ;;
        tracking_enabled)
        set_tracking ${argv[++j]}
        ;;
        bsm_enabled)
        set_bsm ${argv[++j]}
        ;;
        reset)
        reset_cfg
        ;;
        *)
        print_help
        ;;
    esac
done

echo "Done configuration!"
