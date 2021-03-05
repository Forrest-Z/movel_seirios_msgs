cmake_minimum_required(VERSION 3.0.2)
project(map_swapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  multi_map_server
)

catkin_package(
#  INCLUDE_DIRS include
# #  LIBRARIES map_swapper
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
# #  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_swapper src/map_swapper.cpp)
target_link_libraries(map_swapper ${catkin_LIBRARIES})
add_dependencies(map_swapper multi_map_server_gencpp)