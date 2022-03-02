cmake_minimum_required(VERSION 3.16.3)
project(movel_aruco_tools LANGUAGES CXX)

# include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
# include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2_ros
    tf2
    geometry_msgs
    visualization_msgs
    tf
    fiducial_msgs
    movel_seirios_msgs
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
#  LIBRARIES movel_aruco_tools
#  CATKIN_DEPENDS actionlib_msgs geometry_msgs nav_msgs roscpp
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_saver src/aruco_saver.cpp)
add_dependencies(aruco_saver ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_saver ${catkin_LIBRARIES})

add_executable(aruco_amcl src/aruco_amcl.cpp)
add_dependencies(aruco_amcl ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_amcl ${catkin_LIBRARIES})

install(TARGETS aruco_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS aruco_amcl
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()