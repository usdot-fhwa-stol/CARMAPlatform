
# Copyright (C) 2020-2021 LEIDOS.
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
project(carma_wm_ctrl)

## Compile as C++14, as it is required by lanelet2
add_compile_options(-std=c++14)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Set catkin dependencies
set ( PKG_CATKIN_DEPS 
  autoware_lanelet2_msgs
  lanelet2_core
  lanelet2_maps
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_extension
  carma_utils
  carma_wm
  cav_msgs
  roscpp
)

## Find required catkin packages
find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPS}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Catkin export configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PKG_CATKIN_DEPS}
  DEPENDS Boost EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare C++ library
add_library(${PROJECT_NAME}
  src/WMBroadcaster.cpp
  src/GeofenceScheduler.cpp
  src/GeofenceSchedule.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

message(INFO "LIBS: " ${catkin_LIBRARIES})

## Create C++ Node Executable
add_executable(${PROJECT_NAME}_node 
  src/main.cpp 
  src/WMBroadcasterNode.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install Other Resources
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gmock(${PROJECT_NAME}-test
 test/TestMain.cpp
 test/GeofenceSchedulerTest.cpp
 test/GeofenceScheduleTest.cpp
 test/WMBroadcasterTest.cpp
 test/MapToolsTest.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)

catkin_add_gmock(map-tools
 test/TestMain.cpp
 test/MapToolsTest.cpp
 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test # Add test directory as working directory for unit tests
)

if(TARGET ${PROJECT_NAME}-test)
 target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

if(TARGET map-tools)
 target_link_libraries(map-tools ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
