cmake_minimum_required(VERSION 3.16.3)
project(docking_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  ros_utils
  roscpp
  rospy
  std_msgs
  camera_lidar_docking_pallet
  movel_seirios_msgs
  movel_common_libs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES docking_handler
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../task_supervisor/include
)

add_library(docking_handler src/docking_handler.cpp)
add_dependencies(docking_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(docking_handler ${catkin_LIBRARIES})


install(TARGETS docking_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES docking_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()