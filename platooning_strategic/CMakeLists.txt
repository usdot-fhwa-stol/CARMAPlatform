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
project(platoon_strategic)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  carma_utils
  cav_msgs
  cav_srvs
  roscpp
  std_msgs
  carma_wm
  lanelet2_extension
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS carma_utils cav_msgs roscpp std_msgs carma_wm cav_srvs lanelet2_extension
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
)

file(GLOB_RECURSE headers */*.hpp */*.h)

add_executable( ${PROJECT_NAME}
  ${headers}
  src/platoon_strategic.cpp
  src/platoon_manager.cpp
  src/main.cpp)


add_library(platoon_strategic_plugin_lib
 src/platoon_strategic.cpp
 src/platoon_manager.cpp)

add_dependencies(platoon_strategic_plugin_lib ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} platoon_strategic_plugin_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#############
## Install ##
#############


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install C++
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
  test/test_platoon_manager.cpp
  test/mobility_messages.cpp
  test/main_test.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test platoon_strategic_plugin_lib ${catkin_LIBRARIES})
endif()
