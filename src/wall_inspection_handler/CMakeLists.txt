cmake_minimum_required(VERSION 3.16.3)
project(wall_inspection_handler LANGUAGES CXX)

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
    movel_common_libs
    task_supervisor
    geometry_msgs
    movel_seirios_msgs
)

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES wall_inspection_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 #${boost_INCLUDE_DIRS}
 #../task_supervisor/include
)

add_library(wall_inspection_handler src/wall_inspection_handler.cpp)
add_dependencies(wall_inspection_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_inspection_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS wall_inspection_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES wall_inspection_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()