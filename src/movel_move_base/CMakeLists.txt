cmake_minimum_required(VERSION 3.0.2)
project(movel_move_base)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS  
  actionlib
  base_local_planner
  clear_costmap_recovery
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  rospy
  rotate_recovery
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  movel_hasp_vendor
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# license
if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
  add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
  add_definitions(-DMOVEL_LICENSE)
endif()

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MoveBase.cfg
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES 
    movel_move_base
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

### BUILD

# move_base
add_library(movel_move_base
  src/move_base.cpp
  src/move_base_utils.cpp
)
target_link_libraries(movel_move_base
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(movel_move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_node
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node movel_move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

### INSTALL

install(TARGETS move_base_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS movel_move_base
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()