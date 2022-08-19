cmake_minimum_required(VERSION 3.16.3)
project(kudan_localization_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    movel_seirios_msgs
    kdlidar_ros_msgs
    pluginlib
    std_srvs
    ros_utils
    roscpp
    rospy
    std_msgs
    task_supervisor
    ndt_omp
    hdl_graph_slam
    fast_gicp
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kudan_localization_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(kudan_localization_handler src/kudan_localization_handler.cpp)
add_dependencies(kudan_localization_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kudan_localization_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS kudan_localization_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES kudan_localization_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()