cmake_minimum_required(VERSION 3.16.3)
project(wait_for_go_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pluginlib
  movel_seirios_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wait_for_go_handler
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../task_supervisor/include
)

add_library(wait_for_go_handler src/wait_for_go.cpp)
add_dependencies(wait_for_go_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wait_for_go_handler ${catkin_LIBRARIES})

install(TARGETS wait_for_go_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
