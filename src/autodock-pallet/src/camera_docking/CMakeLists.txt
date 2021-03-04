cmake_minimum_required(VERSION 3.16.3)
project(camera_docking LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  tf
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()

find_package(OpenCV 4.2 REQUIRED)

#generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

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

add_library(camera_dock src/camera_docking.cpp)
add_dependencies(camera_dock ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_dock ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(camera_docking src/camera_docking_node.cpp)
add_dependencies(camera_docking camera_dock ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_docking camera_dock ${catkin_LIBRARIES} ${OpenCV_LIBS})
#############
## Install ##
#############


install(TARGETS camera_dock
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS camera_docking
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


