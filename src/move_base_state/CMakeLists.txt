cmake_minimum_required(VERSION 3.16.3)
project(move_base_state LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosgraph_msgs
  movel_hasp_vendor
  ros_utils
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
      add_definitions(-DMOVEL_LICENSE)
endif()


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_base_state src/move_base_state.cpp)
target_link_libraries(move_base_state ${catkin_LIBRARIES})
add_dependencies(move_base_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_move_base_state test/test_move_base_state.test test/test_move_base_state.cpp)
  target_link_libraries(test_move_base_state ${catkin_LIBRARIES})
endif()

install(TARGETS move_base_state
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
