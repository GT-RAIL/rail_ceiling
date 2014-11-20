cmake_minimum_required(VERSION 2.8.3)
project(rail_ceiling)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ar_track_alvar_msgs geometry_msgs roscpp roslib tf)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  Obstacle.msg
  Obstacles.msg
)

add_service_files(
  FILES
  GetAllObstacles.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Check for what version of YAML we have
add_definitions(-DYAMLCPP_GT_0_5_0)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

## Declare a cpp executable
add_executable(calibration src/calibration.cpp)
add_executable(calibration_from_carl src/calibration_from_carl.cpp)
add_executable(furniture_tracker src/furniture_tracker.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(calibration
  ${catkin_LIBRARIES}
)
target_link_libraries(calibration_from_carl
  ${catkin_LIBRARIES}
)
target_link_libraries(furniture_tracker
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_dependencies(furniture_tracker ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS calibration calibration_from_carl furniture_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Copy config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Copy rviz files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## Copy urdf files
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

## Copy marker files
install(DIRECTORY markers/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/markers
)

## Copy bundle files
install(DIRECTORY bundles/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bundles
)
