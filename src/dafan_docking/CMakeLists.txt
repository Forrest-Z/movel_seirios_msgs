cmake_minimum_required(VERSION 3.0.2)
project(dafan_docking)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  dafan_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  apriltag_ros
  move_base_msgs 
  actionlib
  dynamic_reconfigure
)

#find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  DIRECTORY
    srv
  FILES
    ToggleDocking.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/planner_adjuster.cfg
  cfg/april_docking.cfg
  cfg/orientate_plane.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
    nav_msgs sensor_msgs tf2_geometry_msgs dafan_msgs move_base_msgs 
    actionlib
    tf2 tf2_ros 
    dynamic_reconfigure 
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(dafan_simple_pid src/simple_pid.cpp)
target_link_libraries(dafan_simple_pid ${catkin_LIBRARIES})

#EXECUTABLES

add_executable(pre_docking src/pre_docking.cpp)
target_link_libraries(pre_docking ${catkin_LIBRARIES})
add_dependencies(pre_docking ${catkin_EXPORTED_TARGETS})

add_executable(april_docking src/april_docking.cpp)
target_link_libraries(april_docking ${catkin_LIBRARIES})
add_dependencies(april_docking ${PROJECT_NAME}_gencfg )

add_executable(orientate_plane src/orientate_plane.cpp)
target_link_libraries(orientate_plane ${catkin_LIBRARIES})
add_dependencies(orientate_plane ${catkin_EXPORTED_TARGETS})

add_executable(dafan_planner_adjuster src/planner_adjuster.cpp)
target_link_libraries(dafan_planner_adjuster dafan_simple_pid ${catkin_LIBRARIES} )
add_dependencies(dafan_planner_adjuster ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dafan_docking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#
