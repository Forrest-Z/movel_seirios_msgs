cmake_minimum_required(VERSION 3.16.3)
project(camera_lidar_docking_pallet LANGUAGES CXX)

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
  std_srvs
  dynamic_reconfigure
  geometry_msgs
  pcl_ros
  message_generation
  movel_hasp_vendor
  navigation_based_docking
)

if ("$ENV{MOVEL_LICENSE}" STREQUAL "KQAYZ")
      add_definitions(-DMOVEL_LICENSE)
endif()

if ("$ENV{MOVEL_LICENSE}" STREQUAL "RWBBX")
      add_definitions(-DMOVEL_LICENSE)
endif()

find_package(OpenCV 4.2 REQUIRED)


add_service_files(DIRECTORY srv
  FILES
  DropObject.srv
  StartAutoDock.srv
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
        ../navigation_based_docking/include
)

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

#add_library(lidar_dock_pallet src/lidar_docking.cpp)
#target_link_libraries(lidar_dock_pallet ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_library(lidar_dock_concat src/lidar_docking_concatenated_point_cloud.cpp)
target_link_libraries(lidar_dock_concat ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(lidar_dock_concat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(camera_lidar_dock_pallet src/camera_lidar_docking.cpp)
target_link_libraries(camera_lidar_dock_pallet  lidar_dock_concat  ${catkin_LIBRARIES} ${OpenCV_LIBS})  #lidar_dock_pallet
add_dependencies(camera_lidar_dock_pallet lidar_dock_concat ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(camera_lidar_docking_pallet src/camera_lidar_docking_node.cpp)
target_link_libraries(camera_lidar_docking_pallet camera_lidar_dock_pallet ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(camera_lidar_docking_pallet camera_lidar_dock_pallet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#add_executable(drop_object src/drop_object.cpp)
#add_dependencies(camera_lidar_docking_pallet camera_lidar_dock_pallet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(drop_object camera_lidar_dock_pallet ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(camera_test tests/camera_test.cpp)
#target_link_libraries(camera_test
#	${catkin_LIBRARIES} 
#	${OpenCV_LIBS}
#	${Boost_LIBRARIES}
#)
#target_link_libraries(camera_test camera_dock_ camera_lidar_dock_pallet ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})



#add_executable(lidar_test tests/lidar_test.cpp)
#target_link_libraries(lidar_test
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBS}
#        ${Boost_LIBRARIES}
#)
#target_link_libraries(lidar_test lidar_dock_pallet camera_lidar_dock_pallet ${CMAKE_THREAD_LIBS_INIT} gtest gtest_main  ${OpenCV_LIBS})

#############
## Install ##
#############


install(TARGETS lidar_dock_concat camera_lidar_dock_pallet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS camera_lidar_docking_pallet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/cameralidarDock/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/lidar_docking_/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

option(CONFIGURE_DEBIAN_PACKAGE "Configure debian package" OFF)
if (CONFIGURE_DEBIAN_PACKAGE)
  string(REGEX MATCH "^.*/seirios_ros" SEIRIOS_ROOT_DIR ${PROJECT_SOURCE_DIR})
  include(${SEIRIOS_ROOT_DIR}/debian.cmake)
endif()
