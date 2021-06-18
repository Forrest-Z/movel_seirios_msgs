cmake_minimum_required(VERSION 3.16.3)
project(point_cloud_concatenation LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  pcl_ros
  movel_hasp_vendor

)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()


find_package(OpenCV 4.2 REQUIRED)
catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

###########
## Build ##
###########
catkin_package(
INCLUDE_DIRS
include

)
include_directories(
        include
)

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(fuse_pcl src/point_cloud_concatenate.cpp)
add_dependencies(fuse_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fuse_pcl  ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(concatenate_point_cloud src/point_cloud_concatenate_node.cpp)
target_link_libraries(concatenate_point_cloud fuse_pcl ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(concatenate_point_cloud fuse_pcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(camera_docking camera_dock ${catkin_LIBRARIES} ${OpenCV_LIBS})
#############
## Install ##
#############


install(TARGETS fuse_pcl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS concatenate_point_cloud
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

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()