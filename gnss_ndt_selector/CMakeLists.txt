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
project(gnss_ndt_selector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  autoware_msgs
  carma_utils
  cav_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf2 tf2_ros tf2_geometry_msgs autoware_msgs carma_utils cav_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
file(GLOB_RECURSE headers */*.hpp */*.h)
add_executable( ${PROJECT_NAME}
                src/main.cpp)
add_library(gnss_ndt_library src/ndt_reliability_counter.cpp src/localizer_node.cpp)
add_dependencies(${PROJECT_NAME} 
                ${catkin_EXPORTED_TARGETS}
)
add_dependencies(gnss_ndt_library
                ${catkin_EXPORTED_TARGETS}
) 
target_link_libraries(${PROJECT_NAME} gnss_ndt_library ${catkin_LIBRARIES} )

#############
## Install ##
#############

install(DIRECTORY include/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install C++
install(TARGETS ${PROJECT_NAME} gnss_ndt_library
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
catkin_add_gmock(${PROJECT_NAME}-test test/test_gnss_ndt_selector.cpp)
target_link_libraries(${PROJECT_NAME}-test gnss_ndt_library ${catkin_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_gnss_ndt_selector_node test/gnss_ndt_selector.test test/test_gnss_ndt_selector_node.cpp)
  target_link_libraries(test_gnss_ndt_selector_node gnss_ndt_library ${catkin_LIBRARIES})
endif()

