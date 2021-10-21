cmake_minimum_required(VERSION 3.16.3)
project(multi_floor_nav_utils LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  movel_seirios_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_ros
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
  add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
  add_definitions(-DMOVEL_LICENSE)
endif()


generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mfn_save_utils_node src/mfn_save_utils_node.cpp src/mfn_save_utils.cpp)
target_link_libraries(mfn_save_utils_node ${catkin_LIBRARIES})
add_dependencies(mfn_save_utils_node multi_floor_nav_utils_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS mfn_save_utils_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY default_map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/default_map
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()




