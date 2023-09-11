cmake_minimum_required(VERSION 3.16.3)
project(pcl_slam_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    movel_seirios_msgs
    movel_common_libs
    pluginlib
    std_srvs
    ros_utils
    roscpp
    rospy
    std_msgs
    task_supervisor
    pcl_ros
    pcl_conversions
    velodyne_pointcloud
    velodyne_pcl
)

find_package(Boost REQUIRED COMPONENTS system)
#find_package(Sophus REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pcl_slam_handler
 #DEPENDENCIES sophus
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud_saver src/pointcloud_saver.cpp)
add_dependencies(pointcloud_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_saver ${catkin_LIBRARIES}) #${boost_LIBRARIES})

add_library(pcl_slam_handler src/pcl_slam_handler.cpp)
add_dependencies(pcl_slam_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_slam_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS pcl_slam_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pointcloud_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
#
#install(DIRECTORY config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#  PATTERN "*.cfgx" EXCLUDE
#)

install(FILES pcl_slam_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()