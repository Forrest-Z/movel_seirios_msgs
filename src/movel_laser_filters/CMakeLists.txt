#cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.16.3)
project(movel_laser_filters LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#project(movel_laser_filters)

set(CMAKE_CXX_STANDARD 11)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS 
  sensor_msgs 
  roscpp 
  tf 
  filters 
  message_filters
  laser_geometry 
  pluginlib 
  angles 
  message_generation 
  dynamic_reconfigure
)
find_package(catkin REQUIRED COMPONENTS 
  ros_utils
  yaml_utils
  movel_hasp_vendor
  ${THIS_PACKAGE_ROS_DEPS}
)
find_package(Boost REQUIRED COMPONENTS 
  system 
#  signals
)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_message_files(
  FILES
  RackLeg.msg
  RackLegAngular.msg
)

add_service_files(
  FILES
  RackFilterUpdate.srv
  RackAngularFilterUpdate.srv
  ProfileUpdate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

##############################################################################
# Define package
##############################################################################

generate_dynamic_reconfigure_options(
  cfg/IntensityFilter.cfg
  cfg/PolygonFilter.cfg
  cfg/ScanShadowsFilter.cfg
  cfg/SpeckleFilter.cfg
  cfg/AngularBoundsFilter.cfg
  cfg/BoxFilter.cfg
  cfg/AngularBoundsFilterInPlace.cfg
  cfg/FootprintFilter.cfg
  cfg/RangeFilter.cfg
  cfg/CircularFilter.cfg
  cfg/BoxArrayFilter.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_filters laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS} message_runtime
  DEPENDS
)

##############################################################################
# Build
##############################################################################

add_library(pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(pointcloud_filters ${catkin_LIBRARIES})

add_library(laser_scan_filters
  src/laser_scan_filters.cpp
  src/median_filter.cpp
  src/array_filter.cpp
  src/box_filter.cpp
  src/circular_filter.cpp
  src/polygon_filter.cpp
  src/speckle_filter.cpp
  src/intensity_filter.cpp
  src/box_array_filter.cpp
)
add_dependencies(laser_scan_filters ${${PROJECT_NAME}_EXPORTED_TARGETS})   # wait for dynamic_reconfigure cfg headers generation
target_link_libraries(laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# add_dependencies(laser_scan_filters ${PROJECT_NAME}_gencfg)

add_executable(scan_to_cloud_filter_chain src/scan_to_cloud_filter_chain.cpp)
target_link_libraries(scan_to_cloud_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scan_to_scan_filter_chain src/scan_to_scan_filter_chain.cpp)
target_link_libraries(scan_to_scan_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(generic_laser_filter_node src/generic_laser_filter_node.cpp)
target_link_libraries(generic_laser_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##############################################################################
# Rack filters
##############################################################################

add_executable(rack_filter src/rack_filter.cpp)
add_dependencies(rack_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rack_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(rack_angular_filter src/rack_angular_filter.cpp)
add_dependencies(rack_angular_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rack_angular_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#######################
# Dynamic Laser Client#
#######################

add_executable(dynamic_scan_to_scan_filter_chain src/dynamic_scan_to_scan_filter_chain.cpp)
add_dependencies(dynamic_scan_to_scan_filter_chain ${PROJECT_NAME}_gencpp) #gencpp is for message/service generation
target_link_libraries(dynamic_scan_to_scan_filter_chain 
                      ${catkin_LIBRARIES}
                      yaml-cpp
                      ${Boost_LIBRARIES})
# add_dependencies(dynamic_scan_to_scan_filter_chain movel_laser_filters_gencpp) #gencpp is for message/service generation

add_executable(laserscanmultifilter_node src/laserscanmultifilter_node.cpp)
add_dependencies(laserscanmultifilter_node ${PROJECT_NAME}_gencpp)
target_link_libraries(laserscanmultifilter_node
                      yaml-cpp
                      ${catkin_LIBRARIES} 
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})

##################
# Service clients#
##################

add_executable(profile_update_client src/profile_update_client.cpp)
add_dependencies(profile_update_client ${PROJECT_NAME}_gencpp)
target_link_libraries(profile_update_client 
                      ${catkin_LIBRARIES})
# add_dependencies(profile_update_client movel_laser_filters_gencpp)

##########
# Testing#
##########

if (CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_executable(test_scan_filter_chain test/test_scan_filter_chain.cpp)
  target_link_libraries(test_scan_filter_chain laser_scan_filters ${rostest_LIBRARIES} ${GTEST_LIBRARIES})
  add_dependencies(test_scan_filter_chain gtest)

  add_rostest(test/test_scan_filter_chain.launch)
  add_rostest(test/test_polygon_filter.launch)
  add_rostest(test/test_speckle_filter.launch)

  catkin_add_gtest(test_shadow_detector test/test_shadow_detector.cpp)
  target_link_libraries(test_shadow_detector ${catkin_LIBRARIES} ${rostest_LIBRARIES})
endif()

##############################################################################
# Install
##############################################################################

install(TARGETS 
  pointcloud_filters 
  laser_scan_filters 
  scan_to_cloud_filter_chain
  scan_to_scan_filter_chain
  generic_laser_filter_node
  dynamic_scan_to_scan_filter_chain
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES movel_laser_filters_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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

