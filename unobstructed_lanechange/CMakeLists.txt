# Copyright (C) 2018-2020 LEIDOS.
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

cmake_minimum_required(VERSION 2.8.3)
project(unobstructed_lanechange)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

set(CATKIN_DEPS
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
  basic_autonomy
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
 ${CATKIN_DEPS}
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPS}
  DEPENDS Boost EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_executable(${PROJECT_NAME}
  src/main.cpp)


add_library(unobstructed_lanechange_plugin_lib
src/unobstructed_lanechange.cpp
)

add_dependencies(unobstructed_lanechange_plugin_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} unobstructed_lanechange_plugin_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(lc_helper test/helper.cpp)
target_link_libraries(lc_helper
  ${catkin_LIBRARIES}
)

add_executable(lc_helper_node test/helper.cpp)
target_link_libraries(lc_helper_node
  lc_helper
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############


install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} unobstructed_lanechange_plugin_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############
catkin_add_gmock(${PROJECT_NAME}-test
 test/test_unobstructed_lanechange_plugin.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test)
target_link_libraries(${PROJECT_NAME}-test unobstructed_lanechange_plugin_lib ${catkin_LIBRARIES})

# if(CATKIN_ENABLE_TESTING)
#   find_package(rostest REQUIRED)
#   add_rostest_gtest(rostest_ulc test/ulc_plugin.test test/rostest_ulc.cpp)
#   target_link_libraries(rostest_ulc ${catkin_LIBRARIES})
# endif()