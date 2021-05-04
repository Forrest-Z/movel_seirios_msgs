cmake_minimum_required(VERSION 3.16.3)
project(dafan_msgs LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  dynamic_reconfigure
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/device_attributes.cfg
)

add_message_files(
  DIRECTORY 
    msg
  FILES
    BatteryStatus.msg
    SwitchStatus.msg
    RobotStatus.msg
    ToggleStatus.msg
    SonarScan.msg
    Motor.msg
    SonarThreshold.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    ToggleDevices.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
  dynamic_reconfigure
)

install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
