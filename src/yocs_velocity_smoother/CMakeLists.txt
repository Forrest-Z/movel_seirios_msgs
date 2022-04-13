cmake_minimum_required(VERSION 3.16.3)
project(yocs_velocity_smoother LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

find_package(catkin REQUIRED COMPONENTS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/params.cfg)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}_nodelet
   CATKIN_DEPENDS roscpp pluginlib nodelet ecl_threads dynamic_reconfigure geometry_msgs nav_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include ${catkin_INCLUDE_DIRS})


# Nodelet library
add_library(${PROJECT_NAME}_nodelet src/velocity_smoother_nodelet.cpp)

# <------------ add hiredis dependency --------------->
find_path(HIREDIS_HEADER hiredis)
target_include_directories(${PROJECT_NAME}_nodelet PUBLIC ${HIREDIS_HEADER})

find_library(HIREDIS_LIB hiredis)
target_link_libraries(${PROJECT_NAME}_nodelet ${HIREDIS_LIB})

# <------------ add redis-plus-plus dependency -------------->
# NOTE: this should be *sw* NOT *redis++*
find_path(REDIS_PLUS_PLUS_HEADER sw)
target_include_directories(${PROJECT_NAME}_nodelet PUBLIC ${REDIS_PLUS_PLUS_HEADER})

find_library(REDIS_PLUS_PLUS_LIB redis++)
target_link_libraries(${PROJECT_NAME}_nodelet ${REDIS_PLUS_PLUS_LIB})


target_link_libraries(${PROJECT_NAME}_nodelet ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_gencfg)


install(TARGETS ${PROJECT_NAME}_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()