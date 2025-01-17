cmake_minimum_required(VERSION 3.16.3)
project(reg_pure_pursuit_local_planner LANGUAGES CXX)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        nav_core
        nav_msgs
        pluginlib
        pcl_conversions
        roscpp
        tf
        geometry_msgs
        tf2
        tf2_geometry_msgs
        tf2_ros
        base_local_planner
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
add_definitions(${Eigen3_DEFINITIONS})

link_directories(${catkin_LIBRARY_DIRS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/RegPurePursuitPlanner.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES reg_pure_pursuit_local_planner
        CATKIN_DEPENDS
        dynamic_reconfigure
        pluginlib
        roscpp
)

add_library(reg_pure_pursuit_local_planner
        src/reg_pure_pursuit_planner.cpp
        src/transform_global_plan.cpp
        src/collision_checker.cpp)

target_link_libraries(reg_pure_pursuit_local_planner ${catkin_LIBRARIES})
add_dependencies(reg_pure_pursuit_local_planner reg_pure_pursuit_local_planner_gencfg)
# add_dependencies(pure_pursuit_local_planner )

install(TARGETS reg_pure_pursuit_local_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES reg_pure_pursuit_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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