cmake_minimum_required(VERSION 3.16.3)
project(human_detection LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
              roscpp 
              std_msgs 
              sensor_msgs 
              visualization_msgs 
              geometry_msgs 
              pcl_conversions pcl_ros tf ros_utils laser_geometry
              dynamic_reconfigure
              movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()


find_package(PCL REQUIRED)

#dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/human_detection.cfg)

add_message_files(FILES ClusterArray.msg)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES human_detection
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS})


add_library(human_detection src/human_detection.cpp)
add_dependencies(human_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(human_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(human_detection_node src/human_detection_node.cpp)
target_link_libraries(human_detection_node ${catkin_LIBRARIES} human_detection)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(human_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_HumanDetection_class test/test_HumanDetection_class.test test/test_HumanDetection_class.cpp)
  target_link_libraries(test_HumanDetection_class ${catkin_LIBRARIES} human_detection)

  add_rostest_gtest(test_human_detection_node test/test_human_detection_node.test test/test_human_detection_node.cpp)
  target_link_libraries(test_human_detection_node ${catkin_LIBRARIES} human_detection)
endif()

install(TARGETS human_detection human_detection_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}/config
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig1.cfgx)")
install(CODE "execute_process(COMMAND linuxenv -v:/home/$ENV{USER}/vendor.hvc --cfg:../envconfig2.cfgx)")

