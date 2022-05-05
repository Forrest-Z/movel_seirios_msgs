cmake_minimum_required(VERSION 3.16.3)
project(plan_inspector LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  roscpp
  tf2_ros
  tf2
  dynamic_reconfigure
  rosgraph_msgs
  std_msgs
  move_base_msgs
  movel_hasp_vendor
  movel_seirios_msgs
  pluginlib
  nav_core
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

generate_dynamic_reconfigure_options(
  cfg/PlanInspector.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES plan_inspector
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/plan_inspector.cpp
# )

add_executable(plan_inspector src/plan_inspector_node.cpp)
add_dependencies(plan_inspector ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_inspector ${catkin_LIBRARIES})

add_executable(plan_inspector_common src/plan_inspector_node_common.cpp)
add_dependencies(plan_inspector_common ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(plan_inspector_common ${catkin_LIBRARIES})

add_executable(obstacle_feature src/obstacle_feature.cpp)
add_dependencies(obstacle_feature ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_feature ${catkin_LIBRARIES})

install(TARGETS obstacle_feature
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS plan_inspector plan_inspector_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()