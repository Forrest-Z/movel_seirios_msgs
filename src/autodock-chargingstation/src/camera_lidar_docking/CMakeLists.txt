cmake_minimum_required(VERSION 3.16.3)
project(camera_lidar_docking LANGUAGES CXX)
include(${PROJECT_SOURCE_DIR}/cmake/cxxStandard.cmake)

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
)

find_package(OpenCV 3.2 REQUIRED)

#generate_dynamic_reconfigure_options(cfg/DetectorParams.cfg)

add_service_files(
  FILES
  StartAutoDock.srv
  FinishedDock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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




add_library(lidar_dock_ src/lidar_docking.cpp)
target_link_libraries(lidar_dock_ ${catkin_LIBRARIES} )

add_library(camera_dock_ src/camera_docking.cpp)
#add_dependencies(camera_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_dock_ lidar_dock_ ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_library(camera_lidar_dock src/camera_lidar_docking.cpp)
#add_dependencies(camera_lidar_dock camera_dock lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_lidar_dock camera_dock_ lidar_dock_  ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(camera_lidar_docking src/camera_lidar_docking_node.cpp)
#add_dependencies(camera_lidar_docking camera_lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera_lidar_docking camera_lidar_dock ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(publish_waypoint src/read_waypoints.cpp)
#add_dependencies(camera_lidar_docking camera_lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_waypoint  ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(record_waypoints src/record_waypoints.cpp)
#add_dependencies(camera_lidar_docking camera_lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record_waypoints  ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(publish_global_path src/publish_global_path.cpp)
#add_dependencies(camera_lidar_docking camera_lidar_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_global_path  ${catkin_LIBRARIES} ${OpenCV_LIBS})



add_executable(camera_test tests/camera_test.cpp)
target_link_libraries(camera_test
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
)
target_link_libraries(camera_test camera_dock_ camera_lidar_dock ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})



add_executable(lidar_test tests/lidar_test.cpp)
target_link_libraries(lidar_test
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
)
target_link_libraries(lidar_test lidar_dock_ camera_lidar_dock ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS camera_lidar_docking
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
