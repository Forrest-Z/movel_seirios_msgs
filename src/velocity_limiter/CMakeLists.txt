cmake_minimum_required(VERSION 3.16.3)
project(velocity_limiter LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  ros_utils
  yaml_utils
  pcl_ros
  message_generation
  std_srvs
  diagnostic_msgs
  diagnostic_updater
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
      add_definitions(-DMOVEL_LICENSE)
endif()


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES SwitchLimitSet.srv PublishGrid.srv SwitchOperationMode.srv
)

add_message_files(
  DIRECTORY msg
  FILES OperationMode.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

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

add_executable(velocity_limiter_node src/velocity_limiter_node.cpp)
add_dependencies(velocity_limiter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_limiter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS velocity_limiter_node
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
