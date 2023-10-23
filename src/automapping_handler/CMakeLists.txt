cmake_minimum_required(VERSION 3.16.3)
project(automapping_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
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
    movel_common_libs
    task_supervisor
    explore_lite
)

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES automapping_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 #${boost_INCLUDE_DIRS}
 #../task_supervisor/include
)

add_library(automapping_handler src/automapping_handler.cpp)
add_dependencies(automapping_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(automapping_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS automapping_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES automapping_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()