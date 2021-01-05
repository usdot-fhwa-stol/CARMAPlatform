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

# This script takes a system release name and version number as arguments, and 
# updates version dependencies in Dockerfile and /docker/checkout.bash accordingly.

# The -u | --unprompted option can be used to skip the interactive prompts, and
# provide arguments directly from the commandline.

if [[ $# -eq 0 ]]; then
    echo "Enter the system release name:"
    read RELEASE_NAME
    echo "Enter the system release version number:"
    read RELEASE_VERSION
else
    while [[ $# -gt 0 ]]; do
    arg="$1"
    case $arg in
        -u|--unprompted)
            RELEASE_NAME=$2
            RELEASE_VERSION=$3
            shift
            shift
            shift
        ;;
    esac
done
fi

SYSTEM_RELEASE=carma-system-$RELEASE_VERSION
RELEASE_BRANCH=release/$RELEASE_NAME

if git ls-remote -q | grep $RELEASE_BRANCH; then
    echo "Checking out $RELEASE_BRANCH branch."
    git checkout $RELEASE_BRANCH

    echo "Updating .circleci/config.yml base image."
    sed -i "s|autoware.ai:.*|autoware.ai:$SYSTEM_RELEASE|g" .circleci/config.yml

    echo "Updating checkout.bash to point to system release version."
    sed -i "s|CARMA[a-zA-Z]*_[0-9]*\.[0-9]*\.[0-9]*|$SYSTEM_RELEASE|g; s|carma-[a-zA-Z]*-[0-9]*\.[0-9]*\.[0-9]*|$SYSTEM_RELEASE|g" docker/checkout.bash

    echo "Updating Dockerfile to point to system release version."
    sed -i "s|:CARMASystem_[0-9]*\.[0-9]*\.[0-9]*|:$SYSTEM_RELEASE|g; s|:carma-system-[0-9]*\.[0-9]*\.[0-9]*|:$SYSTEM_RELEASE|g; s|:[0-9]*\.[0-9]*\.[0-9]*|:$SYSTEM_RELEASE|g" Dockerfile

    git add docker/checkout.bash Dockerfile 

    git commit -m "Updated dependencies for $SYSTEM_RELEASE"

    git tag -a $SYSTEM_RELEASE -m "$SYSTEM_RELEASE version tag."

    echo "Dockerfile and checkout.bash updated, committed, and tagged."
else
    echo "$RELEASE_BRANCH does not exist. Exiting script."
    exit 0
fi
