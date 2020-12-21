cmake_minimum_required(VERSION 3.16.3)
project(map_updater LANGUAGES CXX)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)


find_package(catkin REQUIRED COMPONENTS
  map_msgs
  nav_msgs
  roscpp
  rospy
  std_srvs
)


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
