cmake_minimum_required(VERSION 2.8.3)
project(rail_ceiling)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED cmake_modules roscpp tf costmap_2d)
find_package(TinyXML REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rail_ceiling
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS TinyXML
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(rail_ceiling src/bundle.cpp src/marker_callback_functor.cpp)
target_link_libraries(rail_ceiling ${TinyXML_LIBRARIES} ${catkin_LIBRARIES})

add_library(ceiling_layer src/ceiling_layer.cpp)

## Declare a cpp executable
add_executable(markers_to_map src/markers_to_map.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(markers_to_map rail_ceiling ${catkin_LIBRARIES} ${TinyXML_LIBRARIES} ${OpenCV_LIBS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rail_ceiling_node rail_ceiling_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark executables and/or libraries for installation
# install(TARGETS rail_ceiling rail_ceiling_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
