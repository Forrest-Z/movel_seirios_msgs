cmake_minimum_required(VERSION 3.0.2)
project(movel_analytics_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  ros_utils
  dynamic_reconfigure
  # movel_seirios_msgs
)

generate_dynamic_reconfigure_options(
  cfg/distance_tracker.cfg
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  LIBRARIES movel_analytics_utils
#  CATKIN_DEPENDS other_catkin_pkg
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

add_library(${PROJECT_NAME} src/distance_tracker.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(distance_tracker_node src/distance_tracker_node.cpp)
target_link_libraries(distance_tracker_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(distance_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} distance_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()
