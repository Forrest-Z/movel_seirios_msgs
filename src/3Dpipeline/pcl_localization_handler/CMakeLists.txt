cmake_minimum_required(VERSION 3.16.3)
project(pcl_localization_handler LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
    movel_seirios_msgs
    pluginlib
    std_srvs
    ros_utils
    roscpp
    rospy
    std_msgs
    task_supervisor
    geometry_msgs
)

#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pcl_localization_handler
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_library(pcl_localization_handler src/pcl_localization_handler.cpp)
add_dependencies(pcl_localization_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pcl_localization_handler ${catkin_LIBRARIES}) #${boost_LIBRARIES})

install(TARGETS pcl_localization_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(FILES pcl_localization_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
