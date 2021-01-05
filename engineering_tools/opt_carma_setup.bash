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

# Script sets up the /opt/carma folder for the user
# Takes in one argument which is the path to the vehicle calibration folder to use

if [ ! -d "$1" ]; then
    echo "Please specify a path to the location of your vehicle calibration folder. Such as carma-config/example_calibration_folder/vehicle/"
    exit -1
fi

GRP_ID=1000
sudo groupadd --gid $GRP_ID carma # create the carma group if it does not already exist and add current user to it
USERNAME=$(whoami)
sudo usermod -a -G $GRP_ID $USERNAME

sudo mkdir -p /opt/carma/logs /opt/carma/.ros /opt/carma/maps /opt/carma/routes /opt/carma/yolo/darknet/data
sudo chgrp -R $GRP_ID /opt/carma/
sudo chmod 775 -R /opt/carma/
sudo chmod 775 /opt/carma/logs /opt/carma/.ros

FILE_EXT=$RANDOM
mkdir ~/carma_temp_$FILE_EXT
cd ~/carma_temp_$FILE_EXT

git clone --branch carma-develop --depth 1 git@github.com:usdot-fhwa-stol/autoware.ai.git
cp -R autoware.ai/core_perception/vision_darknet_detect/darknet/cfg/ /opt/carma/yolo/darknet/cfg/
curl -o /opt/carma/yolo/darknet/data/yolov3.weights -L https://pjreddie.com/media/files/yolov3.weights
chmod 775 /opt/carma/yolo/darknet/data/yolov3.weights

curl -o /opt/carma/maps/pcd_map.pcd -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-config/develop/example_opt_carma/maps/base_map.pcd
chmod 775 /opt/carma/maps/pcd_map.pcd

curl -o /opt/carma/maps/vector_map.osm -L https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/route/resource/map/town01_vector_map_1.osm
chmod 775 /opt/carma/maps/vector_map.osm

curl -o /opt/carma/routes/Test_lanelet111_route_1.csv -L  https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/route/resource/route/Test_lanelet111_route_1.csv
chmod 775 /opt/carma/routes/Test_lanelet111_route_1.csv

ln -s "$1" /opt/carma/vehicle

sudo rm -R ~/carma_temp_$FILE_EXT
