cmake_minimum_required(VERSION 3.16.3)
project(dalu_docking LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  apriltag_ros
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nav_msgs roscpp tf2 tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dalu_docking src/dalu_docking.cpp)
target_link_libraries(dalu_docking ${catkin_LIBRARIES})
#add_dependencies(aruco_pose ${PROJECT_NAME}_gencfg)

add_executable(diff_drive_docking src/diff_drive_docking.cpp)
target_link_libraries(diff_drive_docking ${catkin_LIBRARIES})

add_executable(pose_offset src/pose_offset.cpp)
target_link_libraries(pose_offset ${catkin_LIBRARIES})

add_executable(tag_offset src/tag_offset.cpp)
target_link_libraries(tag_offset ${catkin_LIBRARIES})
#add_dependencies(tag_offset ${PROJECT_NAME}_gencfg)

add_executable(tag_offset_plus src/tag_offset_plus.cpp)
target_link_libraries(tag_offset_plus ${catkin_LIBRARIES})
#add_dependencies(tag_offset ${PROJECT_NAME}_gencfg)

install(TARGETS dalu_docking diff_drive_docking tag_offset tag_offset_plus pose_offset
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()
