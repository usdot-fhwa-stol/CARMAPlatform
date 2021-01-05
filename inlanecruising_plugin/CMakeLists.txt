#
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
#

cmake_minimum_required(VERSION 2.8.3)
project(inlanecruising_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

set( CATKIN_DEPS
  roscpp
  std_msgs
  cav_msgs
  cav_srvs
  carma_utils
  trajectory_utils
  autoware_msgs
  carma_wm
  tf
  tf2
  tf2_geometry_msgs
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
 ${CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS Boost Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable( ${PROJECT_NAME}
  src/main.cpp)
add_library(inlanecruising_plugin_library 
  src/inlanecruising_plugin.cpp
  src/smoothing/CubicSpline.cpp
)
target_link_libraries(inlanecruising_plugin_library ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(inlanecruising_plugin_library ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} inlanecruising_plugin_library ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} inlanecruising_plugin_library ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} inlanecruising_plugin_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Other Resources
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test 
  test/test_inlanecruising_plugin_planning.cpp
  test/test_inlanecruising_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}-test inlanecruising_plugin_library ${catkin_LIBRARIES})


