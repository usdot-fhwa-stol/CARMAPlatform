cmake_minimum_required(VERSION 2.8.3)
project(object_detection_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_msgs
  autoware_msgs
  cav_msgs
  roscpp
  carma_utils
  motion_predict
  tf
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2 

 )

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
#  LIBRARIES object_detection_tracking
 CATKIN_DEPENDS autoware_msgs cav_msgs roscpp carma_utils motion_predict geometry_msgs roscpp rospy std_msgs tf2_msgs wgs84_utils message_filters novatel_gps_msgs carma_utils tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(object_detection_tracking_node src/main.cpp src/object_detection_tracking_node.cpp src/object_detection_tracking_worker.cpp)

add_library(object_detection_tracking_worker_lib src/object_detection_tracking_worker.cpp)
add_dependencies(object_detection_tracking_worker_lib ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(object_detection_tracking_node object_detection_tracking_worker_lib ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables for installation
install(TARGETS object_detection_tracking_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
        launch
        config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
##catkin_add_gmock(${PROJECT_NAME}-test test/test_object.cpp)
##target_link_libraries(${PROJECT_NAME}-test object_detection_tracking_worker_lib ${catkin_LIBRARIES})
