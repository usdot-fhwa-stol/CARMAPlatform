
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

cmake_minimum_required(VERSION 3.0.2)
project(rosbag_mock_drivers)

## Compile as C++14
add_compile_options(-std=c++14)


####################
## Dependancies   ##
####################
set ( PKG_CATKIN_DEPENDS 
  roscpp
  std_msgs
  carma_utils
  cav_msgs
  cav_srvs
)

find_package(catkin REQUIRED COMPONENTS
  ${PKG_CATKIN_DEPENDS}
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS ${PKG_CATKIN_DEPENDS}
 DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MockDriver.cpp
  src/MockDriverNode.cpp
  src/MockCameraDriver.cpp
  src/MockCANDriver.cpp
  src/MockCommsDriver.cpp
  src/MockControllerDriver.cpp
  src/MockGNSSDriver.cpp
  src/MockIMUDriver.cpp
  src/MockLidarDriver.cpp
  src/MockRadarDriver.cpp
  src/MockRoadwaySensorDriver.cpp
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(mock_driver
  src/main.cpp
)


## Add cmake target dependencies of the executable
add_dependencies(mock_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mock_driver
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS mock_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
install(TARGETS mock_driver ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_lidar_test test/mock_lidar.test test/MockLidarDriverROSTest.cpp)
  target_link_libraries(mock_lidar_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_gnss_test test/mock_gnss.test test/MockGNSSDriverROSTest.cpp)
  target_link_libraries(mock_gnss_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_imu_test test/mock_imu.test test/MockIMUDriverROSTest.cpp)
  target_link_libraries(mock_imu_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_camera_test test/mock_camera.test test/MockCameraDriverROSTest.cpp)
  target_link_libraries(mock_camera_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_can_test test/mock_can.test test/MockCANDriverROSTest.cpp)
  target_link_libraries(mock_can_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_radar_test test/mock_radar.test test/MockRadarDriverROSTest.cpp)
  target_link_libraries(mock_radar_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_roadway_sensor_test test/mock_roadway_sensor.test test/MockRoadwaySensorDriverROSTest.cpp)
  target_link_libraries(mock_roadway_sensor_test ${catkin_LIBRARIES})

  find_package(rostest REQUIRED)
  add_rostest_gtest(mock_controller_test test/mock_controller.test test/MockControllerDriverROSTest.cpp)
  target_link_libraries(mock_controller_test ${catkin_LIBRARIES})
endif()
