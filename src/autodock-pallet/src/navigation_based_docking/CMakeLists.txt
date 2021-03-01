cmake_minimum_required(VERSION 3.16.3)
project(navigation_based_docking LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2
  tf
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  std_srvs
  dynamic_reconfigure
  geometry_msgs
  pcl_ros
  message_generation
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()

find_package(OpenCV 4.2 REQUIRED)


add_service_files( DIRECTORY srv
  FILES
  StartAutoDock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs

)

###########
## Build ##
###########

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS message_runtime
DEPENDS OpenCV

)


include_directories(
        include
)

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(navigation_based_docking src/navigation_based_docking.cpp)
target_link_libraries(navigation_based_docking ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(navigation_based_docking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigation_based_docking_node src/navigation_based_docking_node.cpp)
target_link_libraries(navigation_based_docking_node navigation_based_docking ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(navigation_based_docking_node navigation_based_docking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(record_waypoints src/record_waypoints.cpp)
target_link_libraries(record_waypoints  ${catkin_LIBRARIES} )
add_dependencies(record_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(drop_object src/drop_object.cpp)
#add_dependencies(camera_lidar_docking camera_lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(drop_object camera_lidar_dock ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(camera_test tests/camera_test.cpp)
#target_link_libraries(camera_test
#	${catkin_LIBRARIES} 
#	${OpenCV_LIBS}
#	${Boost_LIBRARIES}
#)
#target_link_libraries(camera_test camera_dock_ camera_lidar_dock ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})



#add_executable(lidar_test tests/lidar_test.cpp)
#target_link_libraries(lidar_test
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${Boost_LIBRARIES}
#)
#target_link_libraries(lidar_test lidar_dock_ camera_lidar_dock ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})

#############
## Install ##
#############

install(TARGETS navigation_based_docking
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS navigation_based_docking_node record_waypoints
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
