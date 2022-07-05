cmake_minimum_required(VERSION 3.16.3)
project(speed_limit_zone LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  roscpp
  movel_hasp_vendor
  movel_seirios_msgs
  ros_utils
  tf
  dynamic_reconfigure
  jsk_recognition_msgs
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(throttle_speed src/throttle_speed.cpp)
target_link_libraries(throttle_speed ${catkin_LIBRARIES})
add_dependencies(throttle_speed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(speed_limit_zones src/speed_limit_zones.cpp)
target_link_libraries(speed_limit_zones ${catkin_LIBRARIES})
add_dependencies(speed_limit_zones ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if (CONFIGURE_DEBIAN_PACKAGE)
  # copy python scripts to destination and mark as executables
  install(PROGRAMS scripts/mongo_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else() # will run if user invokes catkin_make or colcon witout -DCONFIGURE_DEBIAN_PACKAGE=ON flag
  catkin_install_python(
    PROGRAMS scripts/mongo_bridge.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(TARGETS throttle_speed
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS speed_limit_zones
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()