cmake_minimum_required(VERSION 3.16.3)
project(pallet_detection LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              std_msgs 
              sensor_msgs 
              visualization_msgs 
              geometry_msgs 
              pcl_conversions pcl_ros tf ros_utils laser_geometry
              movel_hasp_vendor
              tf2
              tf2_ros
              tf2_msgs
              tf2_geometry_msgs
              tf2_sensor_msgs
              move_base_msgs
              apriltag_ros
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()


find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pallet_detection apriltag_detection
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_library(pallet_detection src/pallet_detection.cpp)
add_dependencies(pallet_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(pallet_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(apriltag_detection src/apriltag_detection.cpp)
add_dependencies(apriltag_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(apriltag_detection ${catkin_LIBRARIES})

add_executable(pallet_detection_node src/pallet_detection_node.cpp)
target_link_libraries(pallet_detection_node ${catkin_LIBRARIES} pallet_detection)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(pallet_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

add_executable(apriltag_detection_node src/apriltag_detection_node.cpp)
target_link_libraries(apriltag_detection_node ${catkin_LIBRARIES} apriltag_detection)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(apriltag_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

add_executable(pallet_docking src/pallet_docking.cpp)
target_link_libraries(pallet_docking ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(pallet_docking ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

install(TARGETS pallet_detection pallet_detection_node pallet_docking apriltag_detection apriltag_detection_node
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
