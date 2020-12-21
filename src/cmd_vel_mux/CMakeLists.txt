cmake_minimum_required(VERSION 3.16.3)
project(cmd_vel_mux LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
  ros_utils
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_vel_mux src/cmd_vel_mux.cpp)
target_link_libraries(cmd_vel_mux ${catkin_LIBRARIES})

install(TARGETS cmd_vel_mux
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
