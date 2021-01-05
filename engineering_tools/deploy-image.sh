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

# Docker deployment script for CARMA
#
# Loads a docker image in the local registry to the remote target
# Params:
# $1 - image name to load
# $2 - Username to use on remote host
# $3 - IP address or hostname of remote host
# 
# pv will be used to monitor the progress of the image transfer over the network
# if it is installed. Otherwise no progress will be tracked. pv can be installed
# by running:
#
# sudo apt-get install pv

if [ ! -z $(command -v pv) ]; then
    echo "Installing docker image $1 to host $2@$3..."
    docker save $1 | bzip2 | pv | ssh $2@$3 'bunzip2 | docker load'
    echo "Install complete!"
else 
    echo "Installing docker image $1 to host $2@$3..."
    echo "pv not installed, no progress info available..."
    docker save $1 | bzip2 | ssh $2@$3 'bunzip2 | docker load'
    echo "Install complete!"
fi