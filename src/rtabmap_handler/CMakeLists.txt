cmake_minimum_required(VERSION 3.16.3)
project(rtabmap_handler LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    movel_seirios_msgs
    pluginlib
    std_srvs
    ros_utils
    roscpp
    rospy
    std_msgs
    geometry_msgs
    crop_map
    ipa_building_msgs
    ipa_building_navigation
    ipa_room_exploration
    ipa_room_segmentation
    path_recall
    task_supervisor
    rtabmap_ros
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
  add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
  add_definitions(-DMOVEL_LICENSE)
endif()

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rtabmap_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 #${boost_INCLUDE_DIRS}
 #../task_supervisor/include
)

add_library(rtabmap_handler src/rtabmap_handler.cpp)
add_dependencies(rtabmap_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rtabmap_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS rtabmap_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY launch/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

#install(DIRECTORY config/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#  PATTERN ".svn" EXCLUDE
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES rtabmap_handler_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()