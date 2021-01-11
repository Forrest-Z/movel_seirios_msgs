cmake_minimum_required(VERSION 3.16.3)
project(map_updater LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(catkin REQUIRED COMPONENTS
  map_msgs
  nav_msgs
  roscpp
  rospy
  std_srvs
  movel_hasp_vendor
)
if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
      add_definitions(-DMOVEL_LICENSE)
endif()


catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES map_updater
#  CATKIN_DEPENDS map_msgs nav_msgs roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(map_updater src/map_updater.cpp)
target_link_libraries(map_updater ${catkin_LIBRARIES})

add_executable(map_updater_node src/map_updater_node.cpp)
target_link_libraries(map_updater_node map_updater ${catkin_LIBRARIES})
