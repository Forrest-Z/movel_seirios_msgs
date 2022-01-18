cmake_minimum_required(VERSION 3.16.3)
project(curb_edges_detection LANGUAGES CXX)


include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(PCL 1.2 REQUIRED)
find_package (OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  sensor_msgs
  tf
  tf2_ros
  pcl_ros
  ros_utils
  geometry_msgs
  tf2_geometry_msgs
  movel_seirios_msgs
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES curb_edges_detection 
  CATKIN_DEPENDS roscpp 
  # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(curb_edges_detection src/curb_edges_detection.cpp)
add_dependencies(curb_edges_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(curb_edges_detection ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(curb_edges_detection_node src/curb_edges_detection_node.cpp)
add_dependencies(curb_edges_detection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(curb_edges_detection_node curb_edges_detection ${catkin_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(base_link_flattener src/base_link_flattener.cpp)
target_link_libraries(base_link_flattener ${catkin_LIBRARIES})


add_executable(max_velocity_adjuster src/max_velocity_adjuster.cpp)
add_dependencies(max_velocity_adjuster ${catkin_EXPORTED_TARGETS})
target_link_libraries(max_velocity_adjuster ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS curb_edges_detection curb_edges_detection_node base_link_flattener max_velocity_adjuster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)


option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()
