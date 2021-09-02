cmake_minimum_required(VERSION 3.16.3)
project(movel_octomap_server LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

set(PACKAGE_DEPENDENCIES
  roscpp
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  nodelet
  geometry_msgs
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(cfg/MovelOctomapServer.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS OCTOMAP
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME} src/MovelOctomapServer.cpp src/MovelOctomapServerMultilayer.cpp src/MovelTrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_library(${PROJECT_NAME}_color src/MovelOctomapServer.cpp src/MovelOctomapServerMultilayer.cpp src/MovelTrackingOctomapServer.cpp)
target_link_libraries(${PROJECT_NAME}_color ${LINK_LIBS})
add_dependencies(${PROJECT_NAME}_color ${PROJECT_NAME}_gencfg)
target_compile_definitions(${PROJECT_NAME}_color PUBLIC COLOR_MOVEL_OCTOMAP_SERVER)

add_executable(movel_octomap_server_node src/movel_octomap_server_node.cpp)
target_link_libraries(movel_octomap_server_node ${PROJECT_NAME} ${LINK_LIBS})

add_executable(movel_octomap_color_server_node src/movel_octomap_server_node.cpp)
target_link_libraries(movel_octomap_color_server_node ${PROJECT_NAME}_color ${LINK_LIBS})

add_executable(movel_octomap_server_static src/movel_octomap_server_static.cpp)
target_link_libraries(movel_octomap_server_static ${PROJECT_NAME} ${LINK_LIBS})

add_executable(movel_octomap_server_multilayer src/movel_octomap_server_multilayer.cpp)
target_link_libraries(movel_octomap_server_multilayer ${PROJECT_NAME} ${LINK_LIBS})

add_executable(movel_octomap_saver src/movel_octomap_saver.cpp)
target_link_libraries(movel_octomap_saver ${PROJECT_NAME} ${LINK_LIBS})

add_executable(movel_octomap_tracking_server_node src/movel_octomap_tracking_server_node.cpp)
target_link_libraries(movel_octomap_tracking_server_node ${PROJECT_NAME} ${LINK_LIBS})

# Nodelet
add_library(movel_octomap_server_nodelet src/movel_octomap_server_nodelet.cpp)
target_link_libraries(movel_octomap_server_nodelet ${PROJECT_NAME} ${LINK_LIBS})

add_library(movel_octomap_color_server_nodelet src/movel_octomap_server_nodelet.cpp)
target_link_libraries(movel_octomap_color_server_nodelet ${PROJECT_NAME}_color ${LINK_LIBS})

# install targets:
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_color
  movel_octomap_server_node
  movel_octomap_color_server_node
  movel_octomap_server_static
  movel_octomap_server_multilayer
  movel_octomap_saver
  movel_octomap_tracking_server_node
  movel_octomap_server_nodelet
  movel_octomap_color_server_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(FILES
  nodelet_plugins.xml
  color_nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()