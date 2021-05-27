cmake_minimum_required(VERSION 3.16.3)
project(geometric_docking LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  actionlib_msgs
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dock_detector geometric_docker
  CATKIN_DEPENDS geometry_msgs sensor_msgs tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dock_detector src/dock_detector.cpp)
target_link_libraries(dock_detector ${catkin_LIBRARIES})

add_library(geometric_docker src/geometric_docker.cpp)
target_link_libraries(geometric_docker dock_detector ${catkin_LIBRARIES})

add_executable(geometric_docker_node src/geometric_docker_node.cpp)
add_dependencies(geometric_docker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geometric_docker_node 
  dock_detector
  geometric_docker  
  ${catkin_LIBRARIES}
)

# Directory installs

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

# Target installs

install(TARGETS dock_detector geometric_docker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS geometric_docker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
