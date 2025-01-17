cmake_minimum_required(VERSION 3.16.3)
project(smac_planner LANGUAGES CXX)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

set(CMAKE_BUILD_TYPE Debug) # Debug, Release
set(library_name smac_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

add_compile_options(-O3 -Wextra -Wdeprecated -fPIC)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(${ROS_VERSION} EQUAL 2)

  # Not adding -DUSING_ROS since xml_parsing.cpp hasn't been ported to ROS2

  message(STATUS "------------------------------------------")
  message(STATUS "SMAC Planner is being built using AMENT.")
  message(STATUS "------------------------------------------")

  find_package(ament_cmake REQUIRED)
  find_package(nav2_common REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(nav2_core REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(eigen3_cmake_module REQUIRED)

elseif(${ROS_VERSION} EQUAL 1)

  set(catkin_FOUND 1)
  add_definitions(-DUSING_ROS)
  find_package(catkin REQUIRED COMPONENTS roslib nav_core nav_core2 roscpp
                                          costmap_2d nav_2d_utils dynamic_reconfigure)
  find_package(GTest)

  message(STATUS "------------------------------------------")
  message(STATUS "SMAC Planner is being built using CATKIN.")
  message(STATUS "------------------------------------------")

  generate_dynamic_reconfigure_options(
    cfg/SmacPlanner.cfg
  )

  catkin_package(
    INCLUDE_DIRS
    include # do not include "3rdparty" here
    LIBRARIES
    ${library_name}
    ${library_name}_2d
    CATKIN_DEPENDS
    roslib
    std_msgs
    geometry_msgs
    pluginlib
    tf2_ros
    nav_core
    nav_core2
    roscpp
    visualization_msgs
    nav_msgs
    costmap_2d
    nav_2d_utils
    dynamic_reconfigure)
endif()

include_directories(include ${CERES_INCLUDES} ${OMPL_INCLUDE_DIRS}
                    ${OpenMP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

if(${ROS_VERSION} EQUAL 2)
  set(dependencies
      rclcpp
      rclcpp_action
      rclcpp_lifecycle
      std_msgs
      visualization_msgs
      nav2_util
      nav2_msgs
      nav_msgs
      geometry_msgs
      builtin_interfaces
      tf2_ros
      nav2_costmap_2d
      nav2_core
      pluginlib
      eigen3_cmake_module)

  # SE2 plugin
  add_library(${library_name} SHARED src/nav2_smac_planner.cpp)

  target_link_libraries(${library_name} ${CERES_LIBRARIES} ${OMPL_LIBRARIES}
                        ${OpenMP_LIBRARIES} OpenMP::OpenMP_CXX)
  target_include_directories(${library_name} PUBLIC ${Eigen3_INCLUDE_DIRS})

  ament_target_dependencies(${library_name} ${dependencies})

  # 2D plugin
  add_library(${library_name}_2d SHARED src/nav2_smac_planner_2d.cpp)

  target_link_libraries(${library_name}_2d ${CERES_LIBRARIES} ${OMPL_LIBRARIES})
  target_include_directories(${library_name}_2d PUBLIC ${Eigen3_INCLUDE_DIRS})

  ament_target_dependencies(${library_name}_2d ${dependencies})

  target_compile_definitions(${library_name}
                             PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
  target_compile_definitions(${library_name}_2d
                             PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

  pluginlib_export_plugin_description_file(nav2_core nav2_smac_plugin.xml)
  pluginlib_export_plugin_description_file(nav2_core nav2_smac_plugin_2d.xml)
elseif(${ROS_VERSION} EQUAL 1)
  add_library(footprint_collision_checker src/footprint_collision_checker.cpp)
  add_dependencies(footprint_collision_checker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(footprint_collision_checker
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )

  # 2D plugin
  add_library(${library_name}_2d SHARED src/nav_smac_planner_2d.cpp)

  target_link_libraries(
    ${library_name}_2d
    PRIVATE footprint_collision_checker ${CERES_LIBRARIES} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES}
            OpenMP::OpenMP_CXX ${catkin_LIBRARIES})

  target_include_directories(${library_name}_2d
                             PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_include_directories(
    ${library_name}_2d SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS}
                                     ${catkin_INCLUDE_DIRS})

  add_dependencies(${library_name}_2d ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  # SE2 plugin
  add_library(${library_name} SHARED src/nav_smac_planner.cpp)

  target_link_libraries(
    ${library_name}
    PRIVATE footprint_collision_checker ${CERES_LIBRARIES} ${OMPL_LIBRARIES} ${OpenMP_LIBRARIES}
            OpenMP::OpenMP_CXX ${catkin_LIBRARIES})

  target_include_directories(${library_name}
                             PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
  target_include_directories(
    ${library_name} SYSTEM PUBLIC ${Eigen3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

  add_dependencies(${library_name} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

  install(FILES nav_smac_plugin.xml nav_smac_plugin_2d.xml
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()

if(${ROS_VERSION} EQUAL 2)

  install(
    TARGETS ${library_name} ${library_name}_2d
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY include/ DESTINATION include/)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
  endif()

  ament_export_include_directories(include)
  ament_export_libraries(${library_name} ${library_name}_2d)
  ament_export_dependencies(${dependencies})
  ament_package()
elseif(${ROS_VERSION} EQUAL 1)

  install(
    TARGETS ${library_name} ${library_name}_2d footprint_collision_checker
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
  )

else()
  message(STATUS "ROS is not installed.")
endif()

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()
