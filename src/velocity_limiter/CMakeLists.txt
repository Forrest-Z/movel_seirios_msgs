cmake_minimum_required(VERSION 3.16.3)
project(velocity_limiter LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  movel_seirios_msgs
  laser_geometry
  ros_utils
  yaml_utils
  pcl_ros
  message_generation
  std_srvs
  diagnostic_msgs
  diagnostic_updater
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
# target_include_directories(${PROJECT_NAME} PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
# target_link_libraries(${PROJECT_NAME} ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
# target_include_directories(${PROJECT_NAME} PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)

# add_service_files(
#   DIRECTORY srv
#   FILES SwitchLimitSet.srv PublishGrid.srv SwitchOperationMode.srv
# )

# add_message_files(
#   DIRECTORY msg
#   FILES OperationMode.msg
# )

#generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/common.cpp
  src/velocity_grid.cpp
  src/velocity_limiter.cpp
)

add_library(point_cloud_merger
  src/point_cloud_merger.cpp
)
target_link_libraries(point_cloud_merger ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(velocity_limiter_node src/velocity_limiter_node.cpp)
add_dependencies(velocity_limiter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_include_directories(velocity_limiter_node PUBLIC ${HIREDIS_HEADER})
target_include_directories(velocity_limiter_node PUBLIC ${REDIS_PLUS_PLUS_HEADER})
target_link_libraries(velocity_limiter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${HIREDIS_LIB} ${REDIS_PLUS_PLUS_LIB} ${PROJECT_NAME} yaml-cpp)

add_executable(scan_cloud_merger_node src/scan_cloud_merger_node.cpp)
add_dependencies(scan_cloud_merger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_cloud_merger_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} point_cloud_merger yaml-cpp)

add_executable(point_cloud_merger_node src/point_cloud_merger_node.cpp)
add_dependencies(point_cloud_merger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(point_cloud_merger_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} point_cloud_merger yaml-cpp)

install(TARGETS ${PROJECT_NAME} point_cloud_merger
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS velocity_limiter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS scan_cloud_merger_node point_cloud_merger_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)

catkin_add_gtest(${PROJECT_NAME}-test tests/velocity_limiter_test.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()