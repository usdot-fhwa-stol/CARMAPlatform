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

# This scipt builds the CARMA platform and its Autoware.ai dependancies. 

usage() { 
  echo "USAGE carma_build [OPTION]
  carma_build will build the CARMA Platform including any drivers in the same workspace as well as any required Autoware.ai components
  
  -a Path to Autoware.ai workspace. If this is not specified it is assumed CARMA and Autoware.ai share a workspace 
  -c Path to CARMA workspace.
  -x Skip Autoware.ai build. In this case Autoware.ai will be sourced from the location specified by -a
  -r CARMA rebuild flag. This will do a clean build of the CARMA code. Autoware.ai code is always cleaned before building even without this flag.
  -m Additional build arguments to pass to CARMA's catkin_make install
  -b Additional build arguments to pass to Autoware.ai's colcon build
  -h Print help
  ";
}


# Default environment variables
carma_workspace="$(realpath ../..)"
autoware_src="$(realpath ${carma_workspace}/../autoware.ai)"
skip_autoware=false
rebuild_carma=false
carma_build_args=""
autoware_build_args=""

# Read Options
while getopts a:c:xrhm:b: option
do
	case "${option}"
	in
		a) autoware_src="$(realpath ${OPTARG})";;
		c) carma_workspace="$(realpath ${OPTARG})";;
    x) skip_autoware=true;;
    r) rebuild_carma=true;;
    m) carma_build_args=${OPTARG};;
    b) autoware_build_args=${OPTARG};;
    h) usage; exit 0;;
		\?) echo "Unknown option: -$OPTARG" >&2; exit 1;;
		:) echo "Missing option argument for -$OPTARG" >&2; exit 1;;
		*) echo "Unimplemented option: -$OPTARG" >&2; exit 1;;

	esac
done

echo "
Attempting to build CARMA
CARMA Workspace: ${carma_workspace}
Autoware Source Dir: ${autoware_src}
"

# Clean workspace if needed
old_pwd="${PWD}"
cd ${carma_workspace}

if [ "${rebuild_carma}" = true ]; then
  echo "Clean CARMA build requested with -r option"
  echo "Claning carma workspace"
  rm -rf build devel install
fi

cd ${old_pwd}

###
# Build autoware or skip if requested
###
if [ "${skip_autoware}" = true ]; then
  echo "Skipping Autoware build due to -x option"
  source ${autoware_src}/ros/install/setup.bash
else
  cd ${autoware_src}/autoware/ros
  if [ -z "${autoware_build_args}" ]; then
     ./carma_autoware_build -a ${autoware_src}
  else
     ./carma_autoware_build -a ${autoware_src} -b ${autoware_build_args}
  fi

  source ${autoware_src}/ros/install/setup.bash
  echo "Autoware built successfuly. Binaries sourced from $(realpath ./install/setup.bash)"
fi

###
# Build CARMA 
###
echo "Building CARMA"
cd ${carma_workspace}
colcon build --cmake-args ${carma_build_args}
echo echo "CARMA built successfuly. Binaries sourced from $(realpath ./install/setup.bash)"
