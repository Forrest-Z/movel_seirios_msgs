cmake_minimum_required(VERSION 3.16.3)
project(path_recall LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  ros_utils
  tf
  roslib
  move_base_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES PathInfo.msg
)

add_service_files(
  DIRECTORY srv
  FILES PathName.srv SavePath.srv PathCheck.srv
)

generate_messages(
  DEPENDENCIES std_msgs nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_saver path_recovery path_load_segments
  CATKIN_DEPENDS roscpp std_msgs nav_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(path_saver src/path_saver.cpp)
add_dependencies(path_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(path_recovery src/path_recovery.cpp)
add_dependencies(path_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(path_load_segments src/path_load_segments.cpp)
add_dependencies(path_load_segments ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_saver_node src/path_saver_node.cpp)
target_link_libraries(path_saver_node ${catkin_LIBRARIES} path_saver yaml-cpp)
add_dependencies(path_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_load_segments_node src/path_load_segments_node.cpp)
target_link_libraries(path_load_segments_node ${catkin_LIBRARIES} path_load_segments yaml-cpp)
add_dependencies(path_load_segments_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(path_recovery_node src/path_recovery_node.cpp)
target_link_libraries(path_recovery_node ${catkin_LIBRARIES} path_recovery)
add_dependencies(path_recovery_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(path_recall_test test/path_recall_test.test test/path_recall_test.cpp)
  target_link_libraries(path_recall_test ${catkin_LIBRARIES} path_saver path_load_segments path_recovery yaml-cpp)

  add_rostest_gtest(test_path_recovery_node test/test_path_recovery_node.test test/test_path_recovery_node.cpp)
  target_link_libraries(test_path_recovery_node ${catkin_LIBRARIES} path_recovery)

  add_rostest_gtest(test_path_saver_node test/test_path_saver_node.test test/test_path_saver_node.cpp)
  target_link_libraries(test_path_saver_node ${catkin_LIBRARIES} path_saver)

  add_rostest_gtest(test_path_load_segments_node test/test_path_load_segments_node.test test/test_path_load_segments_node.cpp)
  target_link_libraries(test_path_load_segments_node ${catkin_LIBRARIES} path_load_segments)
endif()

install(TARGETS path_saver path_load_segments path_recovery
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS path_saver_node path_load_segments_node path_recovery_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
