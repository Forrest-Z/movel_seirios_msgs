cmake_minimum_required(VERSION 3.16.3)
project(geometric_docking_handler LANGUAGES CXX)

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
    crop_map
    ipa_room_exploration
    movel_common_libs
    task_supervisor
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES geometric_docking_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${boost_INCLUDE_DIRS}
 #../task_supervisor/include
)

add_library(geometric_docking_handler src/geometric_docking_handler.cpp)
add_dependencies(geometric_docking_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_docking_handler ${catkin_LIBRARIES} ${boost_LIBRARIES})

install(TARGETS geometric_docking_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES geometric_docking_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
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