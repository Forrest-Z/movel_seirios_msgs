cmake_minimum_required(VERSION 3.16.3)
project(camera_lidar_docking_charging_station LANGUAGES CXX)

include(${PROJECT_SOURCE_DIR}/cmake/colorize.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  geometry_msgs
  pcl_ros
  message_generation
  movel_hasp_vendor
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "true")
  add_definitions(-DMOVEL_LICENSE)
endif()


find_package(OpenCV 4.2 REQUIRED)

add_service_files(
  FILES
  StartAutoDock.srv
  FinishedDock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS message_runtime
DEPENDS OpenCV

)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(lidar_dock_charging_station src/lidar_docking.cpp)
target_link_libraries(lidar_dock_charging_station ${catkin_LIBRARIES} )
add_dependencies(lidar_dock_charging_station ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(camera_dock_charging_station src/camera_docking.cpp)
target_link_libraries(camera_dock_charging_station lidar_dock_charging_station ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_dock_charging_station ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(camera_lidar_dock_charging_station src/camera_lidar_docking.cpp)
target_link_libraries(camera_lidar_dock_charging_station camera_dock_charging_station lidar_dock_charging_station  ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_lidar_dock_charging_station camera_dock_charging_station lidar_dock_charging_station ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_lidar_docking_charging_station src/camera_lidar_docking_node.cpp)
target_link_libraries(camera_lidar_docking_charging_station camera_lidar_dock_charging_station ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_lidar_docking_charging_station camera_lidar_dock_charging_station ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test
	${catkin_LIBRARIES} 
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)
target_link_libraries(camera_test camera_dock_charging_station camera_lidar_dock_charging_station ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})



add_executable(lidar_test tests/lidar_test.cpp)
target_link_libraries(lidar_test
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
)
target_link_libraries(lidar_test lidar_dock_charging_station camera_lidar_dock_charging_station ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})

#############
## Install ##
#############

# Install Library
install(TARGETS lidar_dock_charging_station camera_dock_charging_station camera_lidar_dock_charging_station 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install Node
install(TARGETS camera_lidar_docking_charging_station
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Test Node
install(TARGETS camera_test lidar_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/camera_docking_
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/cameralidarDock
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/lidar_docking_
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


