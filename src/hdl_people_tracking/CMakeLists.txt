cmake_minimum_required(VERSION 3.16.3)
project(hdl_people_tracking LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Qt5Gui)
find_package(Qt5Core)
find_package(Qt5Widgets)
find_package(Qt5Test)
find_package(Qt5Concurrent)
find_package(Qt5OpenGL)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf2
  tf2_ros
  nodelet
  pcl_ros
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  roslib
  message_generation
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
  add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
  add_definitions(-DMOVEL_LICENSE)
endif()

########################
## message generation ##
########################
add_message_files(FILES
  Cluster.msg
  ClusterArray.msg
  Track.msg
  TrackArray.msg
)

add_service_files(FILES
  ChooseTarget.srv
  ClearTarget.srv
  Reset.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES hdl_people_tracking
   CATKIN_DEPENDS nodelet pcl_ros roscpp sensor_msgs message_runtime
#   DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#EXECUTABLES

add_executable(velocity_commands src/velocity_commands.cpp)
target_link_libraries(velocity_commands ${catkin_LIBRARIES})
add_dependencies(velocity_commands ${catkin_EXPORTED_TARGETS})


add_library(hdl_people_detection_nodelet
  apps/hdl_people_detection_nodelet.cpp
  src/kidono_human_classifier.cpp
  src/people_detector.cpp
  src/velocity_commands.cpp
)
target_link_libraries(hdl_people_detection_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_detection_nodelet hdl_people_tracking_gencpp)

add_library(hdl_people_tracking_nodelet apps/hdl_people_tracking_nodelet.cpp
	)
target_link_libraries(hdl_people_tracking_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(hdl_people_tracking_nodelet hdl_people_tracking_gencpp)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY include/${PROJECT_NAME}/ include/hdl_people_detection/ include/kkl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS hdl_people_detection_nodelet hdl_people_tracking_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()