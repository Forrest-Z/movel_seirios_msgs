cmake_minimum_required(VERSION 3.16.3)
project(task_supervisor LANGUAGES CXX)

add_compile_options(-std=c++17) # For Multi Floor

# include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
# include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  ros_utils
  # orb_slam2_ros
  pluginlib
  nav_msgs
  flexbe_msgs
  geometry_msgs
  tf
  path_recall
  crop_map
  ipa_room_exploration_msgs
  ipa_room_exploration
  std_msgs
  message_generation
  movel_seirios_msgs
  movel_hasp_vendor
  tf2
  tf2_ros
  tf2_geometry_msgs
  costmap_2d
  movel_fms_utils
  nav_core
  dynamic_reconfigure
  movel_common_libs
  multi_point_navigation
)


find_package(Boost REQUIRED COMPONENTS system filesystem)

find_package(yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES task_handler ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${boost_INCLUDE_DIRS}
)

add_library(task_handler
  src/plugins/task_handler.cpp
)
add_dependencies(task_handler ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(task_handler_plugins
  src/plugins/navigation_handler.cpp
  src/plugins/base/navigation_handler_base.cpp
  src/plugins/navigation_smooth_waypoint_handler.cpp
  src/plugins/navigation_handler_static_clean_path.cpp
  src/plugins/mapping_handler.cpp
  src/plugins/localization_handler.cpp
  # src/base64_codec.cpp
  src/plugins/cleaning_handler.cpp
  # src/plugins/docking_handler.cpp
  # plugins/turn_handler.cpp
  # plugins/lift_handler.cpp
  # plugins/centering_handler.cpp
  src/plugins/human_detection_handler.cpp
  src/plugins/path_handler.cpp
  src/plugins/multi_session_mapping_handler.cpp
  src/plugins/flexbe_handler.cpp
  src/plugins/multi_floor_navigation_handler.cpp
  src/plugins/multi_point_navigation_handler.cpp
  src/plugins/point_based_mapping_handler.cpp
  src/plugins/aruco_handler.cpp
  src/plugins/base/multi_point_navigation_handler_base.cpp
  src/plugins/multi_point_local_planner_navigation_handler.cpp
)

add_dependencies(task_handler_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_handler_plugins
  task_handler
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  yaml-cpp
)

add_library(${PROJECT_NAME} src/server.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME} task_handler task_handler_plugins ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/server_node.cpp src/server.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  task_handler
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  ${PROJECT_NAME}
)


install(TARGETS task_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS task_handler_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY test_files/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test_files
)

install(DIRECTORY test/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN "*.cfgx" EXCLUDE
)

install(FILES task_handler_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CONFIGURE_DEBIAN_PACKAGE)
  # copy python scripts to destination and mark as executables
  install(PROGRAMS scripts/launch_manager_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else() # will run if user invokes catkin_make or colcon witout -DCONFIGURE_DEBIAN_PACKAGE=ON flag
  catkin_install_python(
    PROGRAMS scripts/launch_manager_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig1.cfgx)")

install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig2.cfgx)")

install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig3.cfgx)")

install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig4.cfgx)")

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()