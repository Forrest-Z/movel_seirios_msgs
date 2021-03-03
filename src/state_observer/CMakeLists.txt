cmake_minimum_required(VERSION 2.8.3)
project(state_observer)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  movel_seirios_msgs
  std_msgs
  geometry_msgs
  ros_utils
  message_generation
  diagnostic_msgs
  diagnostic_updater
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_observer src/state_observer.cpp)
add_dependencies(state_observer ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_observer ${catkin_LIBRARIES})

install(TARGETS state_observer
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
