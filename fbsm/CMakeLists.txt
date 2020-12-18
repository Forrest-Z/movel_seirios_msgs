cmake_minimum_required(VERSION 2.8.3)
project(fbsm)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  ros_utils
  movel_license
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

add_message_files(
  FILES
  LineSegment.msg
  LineSegmentList.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES line line_extraction line_extraction_ros feature_description
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp sensor_msgs visualization_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )

add_library(line src/line.cpp)
target_link_libraries(line ${catkin_LIBRARIES})

add_library(line_extraction src/line_extraction.cpp)
target_link_libraries(line_extraction ${catkin_LIBRARIES})

add_library(feature_description src/feature_description.cpp)
target_link_libraries(feature_description ${catkin_LIBRARIES})

add_library(line_extraction_ros src/line_extraction_ros.cpp)
target_link_libraries(line_extraction_ros line line_extraction feature_description ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(line_extraction_ros fbsm_generate_messages_cpp)

add_executable(line_extraction_node src/line_extraction_node.cpp)
target_link_libraries(line_extraction_node line_extraction_ros ${catkin_LIBRARIES})

# catkin_add_gtest(${PROJECT_NAME}-test test/test_fbsm.cpp)

install(TARGETS line line_extraction feature_description line_extraction_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS line_extraction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
