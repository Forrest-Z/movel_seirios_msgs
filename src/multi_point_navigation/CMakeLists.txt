cmake_minimum_required(VERSION 3.0.2)
project(multi_point_navigation)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)


find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
)


generate_dynamic_reconfigure_options(
   cfg/Multipoint.cfg
   cfg/MultipointLocalPlanner.cfg
 )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_point_navigation
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/path_generator.cpp
  src/recovery_behavior_loader.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()